/**
 * imageio.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>
#include <sys/stat.h>
#include <dirent.h>
#include <iostream>

#include "../../Header/Utilities/utils.h"
#include "../../Header/Logging/logger.h"

const std::string LOGGING_NAME = "imageio.cpp";

/**
 * Checks if a file exists at the given path.
 * 
 * @param fileName File path
 * @return true, if file exists
 */
bool fileExists(const std::string &fileName)
{
    struct stat buffer;
    return stat(fileName.c_str(), &buffer) == 0;
}

/**
 * Checks if a directory exists at the given path.
 * 
 * @param dirPath Directory path
 * @return true, if directory exists
 */
bool dirExists(const std::string &dirPath)
{
    struct stat buffer;
    stat(dirPath.c_str(), &buffer);
    return S_ISDIR(buffer.st_mode);
}

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
 * @param image    Puffer für Bildinformationen
 * @param filename Dateipfad des Bildes welches eingelesen werden soll
 * @return true, falls Einlesen erfolgreich war
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
 */
bool loadImages(std::vector<cv::Mat> &images, const std::string &dirPath)
{

    bool exists = dirExists(dirPath);
    bool hasEntries = countEntriesInDir(dirPath) > 0;

    if (dirExists(dirPath) && countEntriesInDir(dirPath) > 0)
    {
        dirent *d;
        DIR *dir = opendir(dirPath.c_str());

        if (dir != NULL)
        {
            while ((d = readdir(dir)) != NULL)
            {
                cv::Mat image;
                std::string dcpp = d->d_name;

                if (loadImage(image, dpp))
                {
                    images.push_back(image);
                }
                else
                {
                    logWarn(LOGGING_NAME, "Image at " + +" couldn't be loaded.");
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
 * Schreibt je nach eingegeben Parameter das neue Bild auf die
 * Standardausgabe oder/und in eine Datei.
 * 
 * @param image    Bildinformationen, also aktuelles Bild
 * @param filename Dateipfad des neuen Bilds
 */
void saveImage(const cv::Mat &image, const std::string &filename)
{
    if (!filename.empty())
    {
        cv::imwrite(filename, image);
    }
}