/**
 * main.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include "../Header/utils.hpp"

/**
 * Specifies whether two strings are equivalent.
 * 
 * @param str1 first string
 * @param str2 second string
 * @return true, if both strings are equivalent
 */
bool equal(const std::string &str1, const std::string &str2)
{
    return !str1.compare(str2);
}