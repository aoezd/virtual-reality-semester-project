/**
 * imageio.cpp
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <iostream>

#include "../../Header/Utilities/utils.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "imageio.cpp";

/**
 * Checks if a file exists at the given path.
 * 
 * @param fileName  File path
 * @return          true, if file exists
 */
bool fileExists(const std::string &fileName)
{
    struct stat buffer;
    return stat(fileName.c_str(), &buffer) == 0;
}

/**
 * Checks if a directory exists at the given path.
 * 
 * @param dirPath   Directory path
 * @return          true, if directory exists
 */
bool dirExists(const std::string &dirPath)
{
    struct stat buffer;
    stat(dirPath.c_str(), &buffer);
    return S_ISDIR(buffer.st_mode);
}

/**
 * Creates a directory at given path, if not existed
 * 
 * @param dirPath   Directory path
 * @return          true, if directory was succesfully created or exists
 */
bool dirMake(const std::string &dirPath) {
    if (!dirExists(dirPath)) {
        return mkdir(dirPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }

    return true;
}

/**
 * Counts all entries in a given directory path.
 * 
 * @param dirPath   Path of directory where entries should be count
 * @return          Counts of entries in a directory
 */
int countEntriesInDir(const std::string &dirPath)
{
    int count = 0;
    dirent *d;
    DIR *dir = opendir(dirPath.c_str());

    if (dir == NULL)
        return 0;
    while ((d = readdir(dir)) != NULL)
    {
        count++;
    }

    closedir(dir);

    return count;
}

/**
 * Lädt ein Bild, welches beim übergebenen Pfad liegt, ein und gibt an
 * ob das Einlesen erfolgreich war.
 * 
 * @param image     Puffer für Bildinformationen
 * @param filename  Path to the image which will be loaded
 * @return          true, falls Einlesen erfolgreich war
 */
bool loadImage(cv::Mat &image, const std::string &filename)
{
    bool exists = fileExists(filename);

    if (exists)
    {
        image = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);

        return true;
    }
    else
    {
        logError(LOGGING_NAME, "File doesn't exists.");
    }

    return false;
}

/**
 * Loads all images in a given directory path.
 * 
 * @param images    Reference to a vector of images
 * @param dirPath   Directory path where all images must be loaded
 * @return          true, if loading was successful
 */
bool loadImages(std::vector<cv::Mat> &images, const std::string &dirPath)
{
    if (dirExists(dirPath) && countEntriesInDir(dirPath) > 0)
    {
        dirent *d;
        DIR *dir = opendir(dirPath.c_str());

        if (dir != NULL)
        {
            while ((d = readdir(dir)) != NULL)
            {
                std::string dcpp = d->d_name;

                if (!equal(dcpp, "..") && !equal(dcpp, "."))
                {
                    cv::Mat image;

                    if (loadImage(image, dirPath + dcpp))
                    {
                        images.push_back(image);
                    }
                    else
                    {
                        logWarn(LOGGING_NAME, "Image at " + dcpp + " couldn't be loaded.");
                    }
                }
            }

            closedir(dir);

            return images.size() > 0;
        }
        else
        {
            logError(LOGGING_NAME, "Directory couldn't be opened.");
        }
    }
    else
    {
        logError(LOGGING_NAME, "Directory doesn't exists or has no entries.");
    }

    return false;
}

/**
 * Saves a given image to a given path.
 * 
 * @param image    Image to save
 * @param filename Path to new image location
 */
void saveImage(const cv::Mat &image, const std::string &filename)
{
    if (!filename.empty())
    {
        cv::imwrite(filename, image);
    }
}