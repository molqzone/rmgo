#include "referee/transfer_registry.hpp"

#include <mutex>
#include <unordered_map>

namespace rmgo_core::referee {
namespace {

using EndpointMap = std::unordered_map<std::string, std::weak_ptr<RefereeTransferEndpoint>>;

std::mutex& registry_mutex() {
    static auto mutex = std::mutex{};
    return mutex;
}

EndpointMap& registry() {
    static auto endpoints = EndpointMap{};
    return endpoints;
}

} // namespace

bool register_referee_transfer_endpoint(
    std::string path, std::shared_ptr<RefereeTransferEndpoint> endpoint) {
    if (path.empty() || endpoint == nullptr) {
        return false;
    }

    const auto lock = std::scoped_lock{registry_mutex()};
    auto& endpoints = registry();
    if (const auto existing = endpoints[path].lock(); existing != nullptr) {
        return existing == endpoint;
    }

    endpoints[std::move(path)] = std::move(endpoint);
    return true;
}

void unregister_referee_transfer_endpoint(
    std::string_view path, const RefereeTransferEndpoint* endpoint) noexcept {
    const auto lock = std::scoped_lock{registry_mutex()};
    auto& endpoints = registry();
    const auto found = endpoints.find(std::string{path});
    if (found == endpoints.end()) {
        return;
    }

    const auto existing = found->second.lock();
    if (endpoint == nullptr || existing == nullptr || existing.get() == endpoint) {
        endpoints.erase(found);
    }
}

std::shared_ptr<RefereeTransferEndpoint> get_referee_transfer_endpoint(std::string_view path) {
    const auto lock = std::scoped_lock{registry_mutex()};
    auto& endpoints = registry();
    const auto found = endpoints.find(std::string{path});
    if (found == endpoints.end()) {
        return nullptr;
    }

    auto endpoint = found->second.lock();
    if (endpoint == nullptr) {
        endpoints.erase(found);
    }
    return endpoint;
}

} // namespace rmgo_core::referee
