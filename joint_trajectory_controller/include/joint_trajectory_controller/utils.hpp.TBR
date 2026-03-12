
#ifndef JOINT_TRAJECTORY_CONTROLLER__UTILS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__UTILS_HPP_


#include <cstddef>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <algorithm>

namespace joint_trajectory_controller
{
namespace utils
{

  
template <typename K, typename V, size_t NumBuckets = 64>
class ConcurrentMap {
    struct Bucket {
        std::unordered_map<K,V> map;
        mutable std::shared_mutex mtx;
    };

    std::vector<Bucket> buckets;

    Bucket& getBucket(const K& key) {
        std::hash<K> h;
        return buckets[h(key) % NumBuckets];
    }

public:
    ConcurrentMap() : buckets(NumBuckets) {}

    void insert_or_assign(const K& key, const V& value) {
        auto& bucket = getBucket(key);
        std::unique_lock lock(bucket.mtx);
        bucket.map[key] = value;
    }

    bool find(const K& key, V& value) const {
        auto& bucket = const_cast<ConcurrentMap*>(this)->getBucket(key);
        std::shared_lock lock(bucket.mtx);
        auto it = bucket.map.find(key);
        if (it == bucket.map.end()) return false;
        value = it->second;
        return true;
    }
    std::size_t size() const { return buckets.size();}
};



}  // namespace utils
}  // namespace joint_trajectory_controller


#endif