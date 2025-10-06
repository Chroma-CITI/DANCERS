#pragma once

#include <yaml-cpp/yaml.h>

// Wrapper function that adds the name of the missing key or the key that generated an error to the error message 

template<typename T>
T getYamlValue(const YAML::Node& node, const std::string& key) {
    try {
        if (!node[key]) {
            throw std::runtime_error("Missing key: " + key);
        }
        return node[key].as<T>();
    } catch (const YAML::TypedBadConversion<T>& e) {
        throw std::runtime_error("Bad conversion for key '" + key + "': " + e.what());
    }
}