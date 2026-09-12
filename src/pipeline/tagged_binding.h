#pragma once

#include <string>
#include <type_traits>
#include <utility>

namespace rvpoint {

/**
 * @brief Categorization of register bindings for DAG analysis and parameter wiring.
 */
enum class TagKind {
  Input,
  Output,
  Parameter
};

/**
 * @brief Input tag marker specifying read-only access to a register slot.
 */
template <typename T>
struct InTag {
  static constexpr TagKind kind = TagKind::Input;
  using type = T;
  using ref_type = const T&;

  std::string name;

  explicit InTag(std::string slot_name) : name(std::move(slot_name)) {}
};

/**
 * @brief Output tag marker specifying write/mutable access to a register slot.
 */
template <typename T>
struct OutTag {
  static constexpr TagKind kind = TagKind::Output;
  using type = T;
  using ref_type = T&;

  std::string name;

  explicit OutTag(std::string slot_name) : name(std::move(slot_name)) {}
};

/**
 * @brief Parameter tag marker specifying configuration values stored in ConfigStore.
 */
template <typename T>
struct ParamTag {
  static constexpr TagKind kind = TagKind::Parameter;
  using type = T;
  using ref_type = T;

  std::string name;
  T default_value;

  explicit ParamTag(std::string param_name, T def_val = T{})
      : name(std::move(param_name)), default_value(std::move(def_val)) {}
};

/**
 * @brief Helper function creating an InTag<T> for input binding.
 */
template <typename T>
inline InTag<T> in(std::string name) {
  return InTag<T>(std::move(name));
}

/**
 * @brief Helper function creating an OutTag<T> for output binding.
 */
template <typename T>
inline OutTag<T> out(std::string name) {
  return OutTag<T>(std::move(name));
}

/**
 * @brief Helper function creating a ParamTag<T> with an optional default value.
 */
template <typename T>
inline ParamTag<T> param(std::string name, T default_val = T{}) {
  return ParamTag<T>(std::move(name), std::move(default_val));
}

// SFINAE detection traits
template <typename T>
struct is_in_tag : std::false_type {};
template <typename T>
struct is_in_tag<InTag<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_in_tag_v = is_in_tag<T>::value;

template <typename T>
struct is_out_tag : std::false_type {};
template <typename T>
struct is_out_tag<OutTag<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_out_tag_v = is_out_tag<T>::value;

template <typename T>
struct is_param_tag : std::false_type {};
template <typename T>
struct is_param_tag<ParamTag<T>> : std::true_type {};
template <typename T>
inline constexpr bool is_param_tag_v = is_param_tag<T>::value;

template <typename T>
inline constexpr bool is_tag_v = is_in_tag_v<T> || is_out_tag_v<T> || is_param_tag_v<T>;

} // namespace rvpoint

