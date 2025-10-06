#ifndef SM_BOOST_PROPERTY_TREE_SUPPORT_HPP
#define SM_BOOST_PROPERTY_TREE_SUPPORT_HPP

#include <filesystem>

namespace sm {

  std::filesystem::path findFile(const std::string& filenameToFind, const std::string& envVarNameContainingSearchDir);

}

#endif /* SM_BOOST_PROPERTY_TREE_SUPPORT_HPP */
