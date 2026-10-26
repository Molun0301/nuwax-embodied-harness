// DUFOMap（BSD-3）在 UFOMap 八叉树上的去动态判定，以 C 接口提供给 Rust。
// 只封装上游 src/dufomap.cpp 的核心步骤：逐帧带传感器原点插入、传播、按“曾被观测为空”判定点；
// 不读写 PCD、不做 toml 配置解析与进度显示。异常不跨越 C 边界。
#include <ufo/map/integration/integration.hpp>
#include <ufo/map/integration/integration_parameters.hpp>
#include <ufo/map/point_cloud.hpp>
#include <ufo/map/ufomap.hpp>

#include <cstdint>
#include <cstring>
#include <exception>
#include <limits>
#include <new>
#include <type_traits>
#include <vector>
#include <cmath>

namespace {
using DufoMapType = ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION>;

struct FreeCell { int32_t x, y, z; uint32_t depth; };

// 只旁观首次空闲迁移，不改变上游积分、保护或传播规则。
struct TrackedMap : DufoMapType {
	using DufoMapType::DufoMapType;
	using DufoMapType::setSeenFree;
	bool tracking = false;
	std::vector<FreeCell> newly_free;
	void setSeenFree(ufo::Index node) {
		if (tracking && !seenFree(node)) {
			auto key = toKey(center(node), depth(node));
			auto zero = toKey(ufo::Point(0, 0, 0));
			newly_free.push_back({static_cast<int32_t>(int64_t(key.x()) - zero.x()),
			                     static_cast<int32_t>(int64_t(key.y()) - zero.y()),
			                     static_cast<int32_t>(int64_t(key.z()) - zero.z()), depth(node)});
		}
		DufoMapType::setSeenFree(node);
	}
};

struct NuwaxDufoMap {
	TrackedMap            map;
	ufo::IntegrationParams params;
	NuwaxDufoMap(double resolution, ufo::depth_t levels) : map(resolution, levels) {}
};

void write_error(char* out, std::size_t len, char const* message)
{
	if (out == nullptr || len == 0) return;
	std::strncpy(out, message, len - 1);
	out[len - 1] = '\0';
}
}  // namespace

extern "C" {

// 对应上游 dufomap 配置及 IntegrationParams；取值含义见 config.toml 与积分参数头文件。
struct nuwax_dufomap_params {
	double   resolution;
	uint32_t levels;
	float    min_range;
	float    max_range;
	uint32_t inflate_unknown;
	uint8_t  inflate_unknown_compensation;
	float    inflate_hits_dist;
	uint8_t  ray_passthrough_hits;
	uint8_t  down_sampling;  // 0 none, 1 center, 2 centroid, 3 uniform
	uint8_t  simple_ray_casting;
	float    simple_ray_casting_factor;
	uint32_t hit_depth;
	uint32_t miss_depth;
	uint8_t  only_valid;
	uint32_t sliding_window_size;
};

void* nuwax_dufomap_new(nuwax_dufomap_params const* p, char* err, std::size_t err_len)
{
	try {
		if (p == nullptr) {
			write_error(err, err_len, "缺少参数");
			return nullptr;
		}
		if (!(p->resolution > 0.0) || p->levels < 2 || p->levels > 21) {
			write_error(err, err_len, "分辨率必须为正，八叉树层数须在2～21之间");
			return nullptr;
		}
		if (p->sliding_window_size > static_cast<uint32_t>(std::numeric_limits<int>::max())) {
			write_error(err, err_len, "滑动窗口帧数超出上游整数范围");
			return nullptr;
		}
		auto* m = new NuwaxDufoMap(p->resolution, static_cast<ufo::depth_t>(p->levels));
		auto& ip = m->params;
		ip.min_range                     = p->min_range;
		ip.max_range                     = p->max_range;
		ip.inflate_unknown               = p->inflate_unknown;
		ip.inflate_unknown_compensation  = p->inflate_unknown_compensation != 0;
		ip.inflate_hits_dist             = p->inflate_hits_dist;
		ip.ray_passthrough_hits          = p->ray_passthrough_hits != 0;
		ip.down_sampling_method          = static_cast<ufo::DownSamplingMethod>(p->down_sampling & 3);
		ip.ray_casting_method            = p->simple_ray_casting ? ufo::RayCastingMethod::SIMPLE
		                                                         : ufo::RayCastingMethod::PROPER;
		ip.simple_ray_casting_factor     = p->simple_ray_casting_factor;
		ip.hit_depth                     = static_cast<ufo::depth_t>(p->hit_depth);
		ip.miss_depth                    = static_cast<ufo::depth_t>(p->miss_depth);
		ip.only_valid                    = p->only_valid != 0;
		ip.sliding_window_size           = static_cast<int>(p->sliding_window_size);
		// 不在库内开线程；并行由调用方按设备核心数决定。
		ip.parallel = false;
		return m;
	} catch (std::exception const& e) {
		write_error(err, err_len, e.what());
	} catch (...) {
		write_error(err, err_len, "创建DUFOMap失败");
	}
	return nullptr;
}

void nuwax_dufomap_free(void* handle) { delete static_cast<NuwaxDufoMap*>(handle); }

void* nuwax_dufomap_clone(void const* handle)
{
	// IntegrationParams 的值复制同时深复制 misses/hits 容器及其环形下标。
	try { return new NuwaxDufoMap(*static_cast<NuwaxDufoMap const*>(handle)); }
	catch (...) { return nullptr; }
}

void nuwax_dufomap_track_free(void* handle) {
	auto& map = static_cast<NuwaxDufoMap*>(handle)->map;
	map.newly_free.clear();
	map.tracking = true;
}

std::size_t nuwax_dufomap_free_cells(void* handle, FreeCell* out, std::size_t capacity) {
	auto& map = static_cast<NuwaxDufoMap*>(handle)->map;
	auto count = map.newly_free.size();
	if (out != nullptr && capacity >= count) {
		std::copy(map.newly_free.begin(), map.newly_free.end(), out);
		map.newly_free.clear();
		map.tracking = false;
	}
	return count;
}

// xyz 为全局坐标系点（n×3 个 float），origin 为同一坐标系的传感器原点。
int nuwax_dufomap_insert(void* handle, float const* xyz, std::size_t n, float const* origin,
                         uint8_t propagate, char* err, std::size_t err_len)
{
	try {
		auto* m = static_cast<NuwaxDufoMap*>(handle);
		if (m == nullptr || (n > 0 && xyz == nullptr) || origin == nullptr) {
			write_error(err, err_len, "无效句柄或输入");
			return -1;
		}
		auto valid = [&](float const* p) {
			return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]) &&
			       m->map.isInside(ufo::Point(p[0], p[1], p[2]));
		};
		// 在积分前拒绝整帧，避免原生断言终止进程或留下半帧状态。
		if (!valid(origin)) { write_error(err, err_len, "雷达原点不在有限八叉树范围内"); return -1; }
		for (std::size_t i = 0; i < n; ++i) {
			if (!valid(xyz + 3 * i)) { write_error(err, err_len, "点坐标不在有限八叉树范围内"); return -1; }
		}
		ufo::PointCloud cloud;
		cloud.reserve(n);
		for (std::size_t i = 0; i < n; ++i) {
			// CloudElement 是以 Point 为基类的聚合类型，只能聚合初始化。
			cloud.push_back(ufo::PointCloud::value_type{ufo::Point(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2])});
		}
		ufo::Point const sensor(origin[0], origin[1], origin[2]);
		// 上游在插入前按最小量程剔除机体附近点（filterminDistance），这里保持同一前置。
		if (m->params.min_range > 0.0f) {
			float const sq = m->params.min_range * m->params.min_range;
			std::erase_if(cloud, [&](auto const& point) { return sensor.squaredDistance(point) < sq; });
		}
		ufo::insertPointCloud(m->map, std::move(cloud), sensor, m->params, propagate != 0);
		return 0;
	} catch (std::exception const& e) {
		write_error(err, err_len, e.what());
	} catch (...) {
		write_error(err, err_len, "插入点云失败");
	}
	return -1;
}

void nuwax_dufomap_propagate(void* handle)
{
	auto* m = static_cast<NuwaxDufoMap*>(handle);
	if (m != nullptr) m->map.propagateModified();
}

// out[i]=1 表示该点所在体素曾被观测为空，即按 DUFOMap 判为动态。
int nuwax_dufomap_seen_free(void const* handle, float const* xyz, std::size_t n, uint8_t* out)
{
	auto const* m = static_cast<NuwaxDufoMap const*>(handle);
	if (m == nullptr || (n > 0 && (xyz == nullptr || out == nullptr))) return -1;
	try {
		for (std::size_t i = 0; i < n; ++i) {
			if (!std::isfinite(xyz[3*i]) || !std::isfinite(xyz[3*i+1]) || !std::isfinite(xyz[3*i+2]) ||
			    !m->map.isInside(ufo::Point(xyz[3*i], xyz[3*i+1], xyz[3*i+2]))) return -1;
			out[i] = m->map.seenFree(xyz[3 * i], xyz[3 * i + 1], xyz[3 * i + 2]) ? 1 : 0;
		}
		return 0;
	} catch (...) {
		return -1;
	}
}

std::size_t nuwax_dufomap_memory_bytes(void const* handle)
{
	auto const* m = static_cast<NuwaxDufoMap const*>(handle);
	return m == nullptr ? 0 : m->map.memoryUsageAllocated();
}

std::size_t nuwax_dufomap_num_nodes(void const* handle)
{
	auto const* m = static_cast<NuwaxDufoMap const*>(handle);
	return m == nullptr ? 0 : m->map.numNodes();
}

std::size_t nuwax_dufomap_window_memory_bytes(void const* handle)
{
	auto const* m = static_cast<NuwaxDufoMap const*>(handle);
	if (m == nullptr) return 0;
	// 仅统计公开容器可确定的容量与有效载荷，避免把实现相关的节点开销冒充精确分配量。
	auto bytes = [](auto const& window) {
		using GridMap = typename std::decay_t<decltype(window)>::value_type;
		std::size_t total = window.capacity() * sizeof(GridMap);
		for (auto const& grids : window) {
			// 默认空 unordered_map 的单桶可内嵌于对象中，不重复计为堆分配。
			if (grids.bucket_count() > 1) total += grids.bucket_count() * sizeof(void*);
			total += grids.size() * sizeof(typename GridMap::value_type);
		}
		return total;
	};
	return bytes(m->params.misses) + bytes(m->params.hits);
}

void nuwax_dufomap_clear(void* handle)
{
	auto* m = static_cast<NuwaxDufoMap*>(handle);
	if (m != nullptr) {
		m->map.clear();
		m->map.newly_free.clear();
		m->map.tracking = false;
		// 窗口随会话重置并释放常驻网格，下一会话不能继承上一会话的射线或命中保护。
		decltype(m->params.misses){}.swap(m->params.misses);
		decltype(m->params.hits){}.swap(m->params.hits);
		m->params.misses_idx = 0;
		m->params.hits_idx = 0;
	}
}
}
