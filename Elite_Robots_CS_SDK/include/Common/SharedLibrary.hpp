#ifndef __ELITE__SHARED_LIBRARY_HPP__
#define __ELITE__SHARED_LIBRARY_HPP__

#include <stdexcept>
#include <string>

namespace ELITE {

class SharedLibrary {
   public:
    /**
     * @brief Constructor.
     * 
     */
    explicit SharedLibrary();

    /**
     * @brief Destructor.
     */
    virtual ~SharedLibrary();

    /**
     * @brief Load library
     * 
     * @param library_path The library string path.
     * @return true 
     * @return false 
     */
    bool loadLibrary(const std::string& library_path);

    /**
     * @brief Unload library
     * 
     */
    bool unloadLibrary();
    
    /**
     * @brief Return true if the shared library contains a specific symbol name otherwise returns false.
     * 
     * @param symbol_name name of the symbol inside the shared library
     * @return if symbols exists returns true, otherwise returns false.
     */
    bool hasSymbol(const std::string& symbol_name);
    
    /**
     * @brief Return shared library symbol pointer.
     * 
     * @param symbol_name name of the symbol inside the shared library
     * @return void* shared library symbol pointer, or nullptr if the symbol doesn't exist.
     */
    void* getSymbol(const std::string& symbol_name);
    
    /**
     * @brief Return shared library path
     * 
     * @return std::string shared library path or it throws an std::runtime_error if it's not defined
     */
    std::string getLibraryPath();

   private:
    void* lib_handle_;
    std::string lib_path_;
};

}  // namespace ELITE

#endif  // __ELITE__SHARED_LIBRARY_HPP__
