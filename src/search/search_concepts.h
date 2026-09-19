#pragma once

#include "core/point_types.h"
#include <type_traits>
#include <utility>
#include <vector>

namespace rvpoint {

// ==============================================================================
// 1. Radius Search Concept Traits (Euclidean R-ball Search)
// ==============================================================================

namespace detail {

template <typename T, typename = void>
struct has_modern_radius_search : std::false_type {};

template <typename T>
struct has_modern_radius_search<
    T,
    std::void_t<decltype(std::declval<const T&>().radius_search(
        0.0f, 0.0f, 0.0f, 0.0f, std::declval<NeighborQueryResult&>()))>>
    : std::true_type {};

template <typename T, typename = void>
struct has_legacy_radius_search : std::false_type {};

template <typename T>
struct has_legacy_radius_search<
    T,
    std::void_t<decltype(std::declval<const T&>().radiusSearch(
        std::declval<const PointXYZ&>(), 0.0f,
        std::declval<std::vector<int>&>(),
        std::declval<std::vector<float>&>()))>>
    : std::true_type {};

template <typename T, typename = void>
struct has_modern_count_neighbors : std::false_type {};

template <typename T>
struct has_modern_count_neighbors<
    T,
    std::void_t<decltype(std::declval<const T&>().count_neighbors(
        0.0f, 0.0f, 0.0f, 0.0f, int{0}))>>
    : std::true_type {};

template <typename T, typename = void>
struct has_legacy_count_neighbors : std::false_type {};

template <typename T>
struct has_legacy_count_neighbors<
    T,
    std::void_t<decltype(std::declval<const T&>().countNeighbors(
        0.0f, 0.0f, 0.0f, 0.0f, int{0}))>>
    : std::true_type {};

template <typename T, typename = void>
struct has_nearest_k_search_impl : std::false_type {};

template <typename T>
struct has_nearest_k_search_impl<
    T,
    std::void_t<decltype(std::declval<const T&>().nearest_k_search(
        0.0f, 0.0f, 0.0f, int{0}, std::declval<NeighborQueryResult&>()))>>
    : std::true_type {};

} // namespace detail

/**
 * @brief Trait checking whether T implements the RadiusSearchable interface.
 */
template <typename T>
struct is_radius_search : std::integral_constant<bool,
    detail::has_modern_radius_search<T>::value ||
    detail::has_legacy_radius_search<T>::value> {};

template <typename T>
inline constexpr bool is_radius_search_v = is_radius_search<T>::value;

/**
 * @brief Trait checking whether T implements count_neighbors short-circuit optimization.
 */
template <typename T>
struct has_neighbor_counting : std::integral_constant<bool,
    detail::has_modern_count_neighbors<T>::value ||
    detail::has_legacy_count_neighbors<T>::value> {};

template <typename T>
inline constexpr bool has_neighbor_counting_v = has_neighbor_counting<T>::value;

/**
 * @brief Trait checking whether T implements the KNNSearchable interface.
 */
template <typename T>
struct is_knn_search : detail::has_nearest_k_search_impl<T> {};

template <typename T>
inline constexpr bool is_knn_search_v = is_knn_search<T>::value;

// ==============================================================================
// 2. Generic Concept Dispatch Helpers
// ==============================================================================

/**
 * @brief Universal radius search helper dispatching to modern or legacy search APIs.
 */
template <typename SearchT>
inline void radius_search(const SearchT& index,
                          float qx, float qy, float qz, float radius,
                          NeighborQueryResult& out) {
    if constexpr (detail::has_modern_radius_search<SearchT>::value) {
        index.radius_search(qx, qy, qz, radius, out);
    } else if constexpr (detail::has_legacy_radius_search<SearchT>::value) {
        std::vector<int> idx;
        std::vector<float> dists;
        index.radiusSearch(PointXYZ{qx, qy, qz}, radius, idx, dists);
        out.clear();
        out.offsets.push_back(0);
        for (int i : idx) {
            out.indices.push_back(static_cast<int32_t>(i));
        }
        out.offsets.push_back(static_cast<uint32_t>(out.indices.size()));
    }
}

/**
 * @brief Universal neighbor counting helper dispatching to fast count or fallback.
 */
template <typename SearchT>
inline int count_neighbors(const SearchT& index,
                           float qx, float qy, float qz, float radius,
                           int max_needed) {
    if constexpr (detail::has_modern_count_neighbors<SearchT>::value) {
        return index.count_neighbors(qx, qy, qz, radius, max_needed);
    } else if constexpr (detail::has_legacy_count_neighbors<SearchT>::value) {
        return index.countNeighbors(qx, qy, qz, radius * radius, max_needed);
    } else {
        NeighborQueryResult res;
        radius_search(index, qx, qy, qz, radius, res);
        int total = static_cast<int>(res.indices.size());
        return total > max_needed ? max_needed : total;
    }
}

} // namespace rvpoint

