/**
 * imageio.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <fstream>
#include <iostream>

#include "../Header/utils.hpp"

const std::string LOGGING_NAME = "imageio.cpp";

/**
 * Prüft ob im übergebenen Pfad eine Datei hinterlegt ist.
 * 
 * @param filename Dateipfad
 * @return true, falls die Datei existiert
 */
bool fileExists(const std::string &filename)
{
    std::ifstream f(filename.c_str());
    return (bool)f;
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
    }
    else
    {
        std::cerr << "ERROR: File doesn't exists." << std::endl;
    }

    return exists;
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