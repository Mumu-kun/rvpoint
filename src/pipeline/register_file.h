#pragma once

#include "core/point_types.h"
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <unordered_map>
#include <variant>
#include <memory>
#include <typeindex>
#include <functional>
#include <stdexcept>
#include <cassert>
#include <type_traits>

namespace rvpoint {

// SFINAE Helpers for Generic Lifecycle Management
template <typename T, typename = void>
struct has_clear : std::false_type {};
template <typename T>
struct has_clear<T, std::void_t<decltype(std::declval<T&>().clear())>> : std::true_type {};
template <typename T>
inline constexpr bool has_clear_v = has_clear<T>::value;

template <typename T, typename = void>
struct has_reserve : std::false_type {};
template <typename T>
struct has_reserve<T, std::void_t<decltype(std::declval<T&>().reserve(std::declval<std::size_t>()))>> : std::true_type {};
template <typename T>
inline constexpr bool has_reserve_v = has_reserve<T>::value;

/**
 * @brief Zero-overhead 16-bit handle for register slots and parameters.
 */
struct RegisterId {
    uint16_t index = 0xFFFF;
    constexpr bool is_valid() const noexcept { return index != 0xFFFF; }
    constexpr bool operator==(RegisterId o) const noexcept { return index == o.index; }
    constexpr bool operator!=(RegisterId o) const noexcept { return index != o.index; }
};

using SlotId = RegisterId;

using ParamValue = std::variant<float, double, int, uint32_t, size_t, bool, std::string>;

enum class RegisterKind { DataSlot, Parameter };

struct RegisterEntry {
    std::string name;
    RegisterKind kind = RegisterKind::DataSlot;
    std::type_index type_id = typeid(void);
    void* instance = nullptr;
    std::unique_ptr<void, void(*)(void*)> storage{nullptr, [](void*){}};
    std::function<void(void*)> reset_fn;
    std::function<void(void*, std::size_t)> reserve_fn;
    ParamValue param_val;
};

/**
 * @brief Slotted Register File consolidating heterogeneous typed slots and parameters.
 *
 * Implements ADR-0012: Zero-copy pointer passing, steady-state zero-heap allocations,
 * and background dynamic parameter updates.
 */
class RegisterFile {
public:
    template <typename T, typename... Args>
    RegisterId get_or_register_slot(const std::string& name, Args&&... args) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) {
            assert(entries_[it->second.index].type_id == typeid(T) && "Register slot type mismatch!");
            return it->second;
        }

        RegisterId id{static_cast<uint16_t>(entries_.size())};
        RegisterEntry entry;
        entry.name = name;
        entry.kind = RegisterKind::DataSlot;
        entry.type_id = typeid(T);

        T* obj = new T(std::forward<Args>(args)...);
        entry.instance = obj;
        entry.storage = std::unique_ptr<void, void(*)(void*)>(obj, [](void* p) {
            delete static_cast<T*>(p);
        });

        if constexpr (has_clear_v<T>) {
            entry.reset_fn = [](void* p) { static_cast<T*>(p)->clear(); };
        }
        if constexpr (has_reserve_v<T>) {
            entry.reserve_fn = [](void* p, std::size_t cap) { static_cast<T*>(p)->reserve(cap); };
        }

        entries_.push_back(std::move(entry));
        name_to_id_[name] = id;
        return id;
    }

    template <typename T>
    RegisterId get_or_register_param(const std::string& name, T default_val) {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) return it->second;

        RegisterId id{static_cast<uint16_t>(entries_.size())};
        RegisterEntry entry;
        entry.name = name;
        entry.kind = RegisterKind::Parameter;
        entry.type_id = typeid(T);
        entry.param_val = ParamValue(default_val);

        entries_.push_back(std::move(entry));
        name_to_id_[name] = id;
        return id;
    }

    RegisterId find(const std::string& name) const noexcept {
        auto it = name_to_id_.find(name);
        if (it != name_to_id_.end()) return it->second;
        return RegisterId{};
    }

    RegisterId get_id(const std::string& name) const {
        auto it = name_to_id_.find(name);
        if (it == name_to_id_.end()) {
            throw std::runtime_error("Register not found in RegisterFile: " + name);
        }
        return it->second;
    }

    template <typename T>
    inline T& get_mut(RegisterId id) noexcept {
        if constexpr (std::is_same_v<T, ParamValue>) {
            return entries_[id.index].param_val;
        } else {
            return *reinterpret_cast<T*>(entries_[id.index].instance);
        }
    }

    template <typename T>
    inline const T& get(RegisterId id) const noexcept {
        if constexpr (std::is_same_v<T, ParamValue>) {
            return entries_[id.index].param_val;
        } else {
            return *reinterpret_cast<const T*>(entries_[id.index].instance);
        }
    }

    template <typename T>
    inline T get_param(RegisterId id) const {
        return std::get<T>(entries_[id.index].param_val);
    }

    template <typename T>
    void set_param(RegisterId id, T val) {
        ParamValue new_v(val);
        if (entries_[id.index].param_val != new_v) {
            entries_[id.index].param_val = new_v;
            dirty_ = true;
            changed_params_.push_back(id);
        }
    }

    template <typename T>
    void set_param(const std::string& name, T val) {
        set_param(get_id(name), val);
    }

    const void* get_pointer(RegisterId id) const noexcept {
        if (entries_[id.index].kind == RegisterKind::Parameter) {
            return &entries_[id.index].param_val;
        }
        return entries_[id.index].instance;
    }

    void reset_frame() {
        for (auto& entry : entries_) {
            if (entry.kind == RegisterKind::DataSlot && entry.reset_fn) {
                entry.reset_fn(entry.instance);
            }
        }
    }

    void preallocate_buffers(std::size_t capacity) {
        for (auto& entry : entries_) {
            if (entry.kind == RegisterKind::DataSlot && entry.reserve_fn) {
                entry.reserve_fn(entry.instance, capacity);
            }
        }
    }

    bool is_dirty() const noexcept { return dirty_; }
    void clear_dirty() noexcept { dirty_ = false; changed_params_.clear(); }
    const std::vector<RegisterId>& changed_params() const noexcept { return changed_params_; }

    std::size_t size() const noexcept { return entries_.size(); }
    const std::string& name(RegisterId id) const noexcept { return entries_[id.index].name; }
    RegisterKind kind(RegisterId id) const noexcept { return entries_[id.index].kind; }

private:
    std::vector<RegisterEntry> entries_;
    std::unordered_map<std::string, RegisterId> name_to_id_;
    bool dirty_ = false;
    std::vector<RegisterId> changed_params_;
};

using ConfigStore = RegisterFile;
using PipelineContext = RegisterFile;

} // namespace rvpoint
