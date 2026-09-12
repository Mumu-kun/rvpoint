#pragma once

#include "core/point_types.h"
#include <type_traits>
#include <utility>

namespace rvpoint {

// ==============================================================================
// Cloud Filter Concept (Non-virtual Functor Protocol)
// ==============================================================================

namespace detail {

template <typename T, typename = void>
struct is_point_cloud_filter_impl : std::false_type {};

template <typename T>
struct is_point_cloud_filter_impl<
    T,
    std::void_t<decltype(std::declval<T&>()(
        std::declval<const PointCloudView&>(),
        std::declval<PointCloud&>()))>>
    : std::true_type {};

} // namespace detail

/**
 * @brief Trait checking whether T implements the PointCloudFilter concept:
 *        void/size_t operator()(const PointCloudView& in, PointCloud& out)
 */
template <typename T>
struct is_point_cloud_filter : detail::is_point_cloud_filter_impl<std::decay_t<T>> {};

template <typename T>
inline constexpr bool is_point_cloud_filter_v = is_point_cloud_filter<T>::value;

/**
 * @brief Free function helper to apply any conforming PointCloudFilter.
 */
template <typename FilterT>
inline auto filter(const PointCloudView& in, PointCloud& out, FilterT&& f) {
    static_assert(is_point_cloud_filter_v<FilterT>,
                  "FilterT must implement operator()(const PointCloudView&, PointCloud&)");
    return f(in, out);
}

/**
 * @brief Convenience overload accepting owning PointCloud as input.
 */
template <typename FilterT>
inline auto filter(const PointCloud& in, PointCloud& out, FilterT&& f) {
    return filter(in.view(), out, std::forward<FilterT>(f));
}

} // namespace rvpoint

