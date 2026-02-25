#pragma once

#include <string>
#include <cstring>

#include <cJSON.h>
#include <esp_heap_caps.h>
#include <nvs_flash.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/http_request/http_request.h"

namespace custom_mww {

static const char *const TAG = "custom_mww";
static const size_t MAX_MODEL_SIZE = 200 * 1024;

// Downloads manifest JSON, parses it, downloads model binary, writes to NVS.
// Returns empty string on success, error message on failure.
inline std::string install(esphome::http_request::HttpRequestComponent *http, const std::string &manifest_url,
                           uint8_t firmware_step_size) {
  // 1. Download manifest JSON
  auto manifest_container = http->get(manifest_url);
  if (manifest_container == nullptr) {
    return "Error: failed to connect to manifest URL";
  }

  size_t manifest_length = manifest_container->content_length;
  if (manifest_length == 0 || manifest_length > 4096) {
    manifest_container->end();
    return "Error: invalid manifest size";
  }

  std::string manifest_json;
  manifest_json.resize(manifest_length);
  size_t total_read = 0;
  while (total_read < manifest_length) {
    int read_len =
        manifest_container->read((uint8_t *) manifest_json.data() + total_read, manifest_length - total_read);
    if (read_len > 0) {
      total_read += read_len;
    } else if (read_len < 0) {
      break;
    }
    esphome::App.feed_wdt();
    esphome::yield();
  }
  manifest_container->end();

  if (total_read != manifest_length) {
    return "Error: incomplete manifest download";
  }

  // 2. Parse manifest JSON
  cJSON *root = cJSON_Parse(manifest_json.c_str());
  if (root == nullptr) {
    return "Error: invalid JSON in manifest";
  }

  cJSON *version_item = cJSON_GetObjectItem(root, "version");
  cJSON *model_item = cJSON_GetObjectItem(root, "model");
  cJSON *wake_word_item = cJSON_GetObjectItem(root, "wake_word");
  cJSON *micro_item = cJSON_GetObjectItem(root, "micro");

  if (!cJSON_IsNumber(version_item) || !cJSON_IsString(model_item) || !cJSON_IsString(wake_word_item) ||
      !cJSON_IsObject(micro_item)) {
    cJSON_Delete(root);
    return "Error: missing required manifest fields";
  }

  int manifest_version = version_item->valueint;
  std::string model_filename = model_item->valuestring;
  std::string wake_word_name = wake_word_item->valuestring;

  uint8_t feature_step_size = 0;
  uint32_t tensor_arena_size = 0;
  float probability_cutoff_f = 0.0f;
  uint16_t sliding_window_size = 0;

  if (manifest_version == 2) {
    cJSON *step_item = cJSON_GetObjectItem(micro_item, "feature_step_size");
    cJSON *arena_item = cJSON_GetObjectItem(micro_item, "tensor_arena_size");
    cJSON *cutoff_item = cJSON_GetObjectItem(micro_item, "probability_cutoff");
    cJSON *window_item = cJSON_GetObjectItem(micro_item, "sliding_window_size");

    if (!cJSON_IsNumber(step_item) || !cJSON_IsNumber(arena_item) || !cJSON_IsNumber(cutoff_item) ||
        !cJSON_IsNumber(window_item)) {
      cJSON_Delete(root);
      return "Error: missing v2 manifest micro fields";
    }

    feature_step_size = (uint8_t) step_item->valueint;
    tensor_arena_size = (uint32_t) arena_item->valueint;
    probability_cutoff_f = (float) cutoff_item->valuedouble;
    sliding_window_size = (uint16_t) window_item->valueint;
  } else if (manifest_version == 1) {
    feature_step_size = 20;
    tensor_arena_size = 45672;

    cJSON *cutoff_item = cJSON_GetObjectItem(micro_item, "probability_cutoff");
    cJSON *window_item = cJSON_GetObjectItem(micro_item, "sliding_window_average_size");

    if (!cJSON_IsNumber(cutoff_item) || !cJSON_IsNumber(window_item)) {
      cJSON_Delete(root);
      return "Error: missing v1 manifest micro fields";
    }

    probability_cutoff_f = (float) cutoff_item->valuedouble;
    sliding_window_size = (uint16_t) window_item->valueint;
  } else {
    cJSON_Delete(root);
    return "Error: unsupported manifest version";
  }

  cJSON_Delete(root);

  // 3. Validate feature_step_size matches firmware
  if (feature_step_size != firmware_step_size) {
    char msg[80];
    snprintf(msg, sizeof(msg), "Error: model step size %u != firmware %u", feature_step_size, firmware_step_size);
    return std::string(msg);
  }

  // 4. Build model URL from manifest base URL + model filename
  std::string model_url = manifest_url;
  size_t last_slash = model_url.rfind('/');
  if (last_slash == std::string::npos) {
    return "Error: cannot determine model URL from manifest URL";
  }
  model_url = model_url.substr(0, last_slash + 1) + model_filename;

  // 5. Download model binary
  ESP_LOGI(TAG, "Downloading model from: %s", model_url.c_str());
  auto model_container = http->get(model_url);
  if (model_container == nullptr) {
    return "Error: failed to connect to model URL";
  }

  size_t model_size = model_container->content_length;
  if (model_size == 0 || model_size > MAX_MODEL_SIZE) {
    model_container->end();
    char msg[64];
    snprintf(msg, sizeof(msg), "Error: invalid model size (%u bytes)", (unsigned) model_size);
    return std::string(msg);
  }

  // Allocate PSRAM buffer for the download
  uint8_t *model_data = (uint8_t *) heap_caps_malloc(model_size, MALLOC_CAP_SPIRAM);
  if (model_data == nullptr) {
    model_container->end();
    return "Error: PSRAM allocation failed";
  }

  total_read = 0;
  while (total_read < model_size) {
    int read_len = model_container->read(model_data + total_read, model_size - total_read);
    if (read_len > 0) {
      total_read += read_len;
    } else if (read_len < 0) {
      break;
    }
    esphome::App.feed_wdt();
    esphome::yield();
  }
  model_container->end();

  if (total_read != model_size) {
    heap_caps_free(model_data);
    return "Error: incomplete model download";
  }

  ESP_LOGI(TAG, "Downloaded %u bytes, writing to NVS", (unsigned) model_size);

  // 6. Write model + metadata to NVS
  uint8_t quantized_cutoff = (uint8_t) (probability_cutoff_f * 255.0f);

  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("mww_user", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    heap_caps_free(model_data);
    return "Error: failed to open NVS";
  }

  nvs_erase_all(nvs_handle);

  bool ok = true;
  ok = ok && (nvs_set_blob(nvs_handle, "model_data", model_data, model_size) == ESP_OK);
  ok = ok && (nvs_set_u32(nvs_handle, "model_size", (uint32_t) model_size) == ESP_OK);
  ok = ok && (nvs_set_u8(nvs_handle, "cutoff", quantized_cutoff) == ESP_OK);
  ok = ok && (nvs_set_u16(nvs_handle, "window", sliding_window_size) == ESP_OK);
  ok = ok && (nvs_set_u32(nvs_handle, "arena", tensor_arena_size) == ESP_OK);
  ok = ok && (nvs_set_u8(nvs_handle, "step", feature_step_size) == ESP_OK);
  ok = ok && (nvs_set_str(nvs_handle, "name", wake_word_name.c_str()) == ESP_OK);
  ok = ok && (nvs_set_str(nvs_handle, "url", manifest_url.c_str()) == ESP_OK);

  if (ok) {
    ok = (nvs_commit(nvs_handle) == ESP_OK);
  }

  nvs_close(nvs_handle);
  heap_caps_free(model_data);

  if (!ok) {
    return "Error: NVS write failed (not enough space?)";
  }

  ESP_LOGI(TAG, "Installed custom wake word '%s'", wake_word_name.c_str());
  return "";
}

// Erases the custom wake word from NVS.
// Returns empty string on success, error message if no model found.
inline std::string remove() {
  nvs_handle_t nvs_handle;
  esp_err_t err = nvs_open("mww_user", NVS_READWRITE, &nvs_handle);
  if (err != ESP_OK) {
    return "No custom wake word installed";
  }

  uint32_t model_size = 0;
  if (nvs_get_u32(nvs_handle, "model_size", &model_size) != ESP_OK || model_size == 0) {
    nvs_close(nvs_handle);
    return "No custom wake word installed";
  }

  nvs_erase_all(nvs_handle);
  nvs_commit(nvs_handle);
  nvs_close(nvs_handle);

  ESP_LOGI(TAG, "Removed custom wake word from NVS");
  return "";
}

// Reads the currently installed wake word name from NVS.
// Returns empty string if none installed.
inline std::string get_installed_name() {
  nvs_handle_t nvs_handle;
  if (nvs_open("mww_user", NVS_READONLY, &nvs_handle) != ESP_OK) {
    return "";
  }

  uint32_t model_size = 0;
  if (nvs_get_u32(nvs_handle, "model_size", &model_size) != ESP_OK || model_size == 0) {
    nvs_close(nvs_handle);
    return "";
  }

  char name[64] = {};
  size_t name_len = sizeof(name);
  nvs_get_str(nvs_handle, "name", name, &name_len);
  nvs_close(nvs_handle);

  return std::string(name);
}

}  // namespace custom_mww
