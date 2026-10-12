// SPDX-License-Identifier: MIT
// Copyright (c) 2025, Elite Robots.

#include "ClassLoader.hpp"
#include "SharedLibrary.hpp"

namespace ELITE {

class ClassLoader::Impl {
   public:
    std::string lib_path_;
    std::unique_ptr<SharedLibrary> shared_library_;
    bool lib_loaded_;
};

ClassLoader::ClassLoader(const std::string& lib_path) {
    impl_ = std::make_unique<Impl>();
    impl_->lib_path_ = lib_path;
    impl_->lib_loaded_ = false;
}

ClassLoader::~ClassLoader() {
    if (impl_ && impl_->shared_library_) {
        // Explicitly unload the shared library before destroying the loader
        impl_->shared_library_->unloadLibrary();
        impl_->shared_library_.reset();
    }
}

bool ClassLoader::loadLib() {
    impl_->shared_library_ = std::make_unique<SharedLibrary>();
    impl_->lib_loaded_ = impl_->shared_library_->loadLibrary(impl_->lib_path_);
    return impl_->lib_loaded_;
}

bool ClassLoader::hasLoadedLib() const { return impl_->lib_loaded_; }

}  // namespace ELITE
