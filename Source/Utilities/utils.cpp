/**
 * main.cpp
 * TODO
 *
 * Created: 2017-07-23
 * Author: Aykut Özdemir
 */

#include <iostream>

#include "../../Header/Utilities/utils.h"

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

// --------------- Printing Stuff ---------------

/**
 *
 */
void printVP2f(const std::vector<cv::Point2f> &vp2f)
{
    std::cout << "[";
    for (size_t i = 0; i < vp2f.size(); i++)
    {
        std::cout << "(" << vp2f[i] << "), ";
    }
    std::cout << '\b' << '\b' << "]," << std::endl;
}

/**
 *
 */
void printVP(const std::vector<cv::Point> &vp)
{
    std::cout << "[";
    for (size_t i = 0; i < vp.size(); i++)
    {
        std::cout << "(" << vp[i] << "), ";
    }
    std::cout << '\b' << '\b' << "]," << std::endl;
}

/**
  *
  */
void printVVP2f(const std::vector<std::vector<cv::Point2f>> &vvp2f)
{
    std::cout << "{" << std::endl;
    for (size_t i = 0; i < vvp2f.size(); i++)
    {
        printVP2f(vvp2f[i]);
    }
    std::cout << '\b' << '\b' << "}" << std::endl;
}

/**
 *
 */
void printVVP(const std::vector<std::vector<cv::Point>> &vvp)
{
    std::cout << "{" << std::endl;
    for (size_t i = 0; i < vvp.size(); i++)
    {
        printVP(vvp[i]);
    }
    std::cout << '\b' << '\b' << "}" << std::endl;
}

// --------------- Converting Stuff ---------------

/**
 *
 */
std::vector<cv::Point2f> convertVP_VP2f(const std::vector<cv::Point> &vp)
{
    std::vector<cv::Point2f> vp2f;

    for (size_t i = 0; i < vp.size(); i++)
    {
        vp2f.push_back(cv::Point2f((float)vp[i].x, (float)vp[i].y));
    }

    return vp2f;
}

/**
 *
 */
std::vector<cv::Point> convertVP2f_VP(const std::vector<cv::Point2f> &vp2f)
{
    std::vector<cv::Point> vp;

    for (size_t i = 0; i < vp2f.size(); i++)
    {
        vp.push_back(cv::Point(vp2f[i].x, vp2f[i].y));
    }

    return vp;
}

/**
 *
 */
std::vector<std::vector<cv::Point2f>> convertVVP_VVP2f(const std::vector<std::vector<cv::Point>> &vvp)
{
    std::vector<std::vector<cv::Point2f>> vvp2f;

    for (size_t i = 0; i < vvp.size(); i++)
    {
        vvp2f.push_back(convertVP_VP2f(vvp[i]));
    }

    return vvp2f;
}

/**
  *
  */
std::vector<std::vector<cv::Point>> convertVVP2f_VVP(const std::vector<std::vector<cv::Point2f>> &vvp2f)
{
    std::vector<std::vector<cv::Point>> vvp;

    for (size_t i = 0; i < vvp2f.size(); i++)
    {
        vvp.push_back(convertVP2f_VP(vvp2f[i]));
    }

    return vvp;
}

/**
 *
 */
std::vector<cv::Point2f> getCornerPoints2f(cv::Mat image)
{
    std::vector<cv::Point2f> c2f{cv::Point2f(image.rows, 0), cv::Point2f(image.rows, image.cols), cv::Point2f(0, image.cols), cv::Point2f(0, 0)};
    return c2f;
}

/**
 *
 */
std::vector<cv::Point> getCornerPoints(cv::Mat image)
{
    std::vector<cv::Point> c{cv::Point(image.rows, 0), cv::Point(image.rows, image.cols), cv::Point(0, image.cols), cv::Point(0, 0)};
    return c;
}

// --------------- Algebra Stuff ---------------

/**
 *
 */
cv::Mat rotate90deg(const cv::Mat &m, const bool &clockwise)
{
    cv::Mat t, f;

    cv::transpose(m, t);
    cv::flip(t, f, clockwise ? 1 : 0);

    return f;
}

/**
 *
 */
cv::Mat rotate90deg(const cv::Mat &m, const bool &clockwise, int count)
{
    if (!count)
    {
        return m;
    }

    cv::Mat t;

    m.copyTo(t);
    for (int i = 0; i < count; i++)
    {
        t = rotate90deg(t, clockwise);
    }

    return t;
}

/**
 * 3 2
 * 0 1
 */
std::vector<cv::Point> rotateQuad90deg(const std::vector<cv::Point> &m, const bool &clockwise)
{
    std::vector<cv::Point> t;

    if (clockwise)
    {
        t.push_back(m[3]);
        t.push_back(m[0]);
        t.push_back(m[1]);
        t.push_back(m[2]);
    }
    else
    {
        t.push_back(m[1]);
        t.push_back(m[2]);
        t.push_back(m[3]);
        t.push_back(m[0]);
    }

    return t;
}

/**
 * 3 2
 * 0 1
 */
std::vector<cv::Point> rotateQuad90deg(const std::vector<cv::Point> &m, const bool &clockwise, int count)
{
    if (!count)
    {
        return m;
    }

    std::vector<cv::Point> t = {m[0], m[1], m[2], m[3]};

    for (int i = 0; i < count; i++)
    {
        t = rotateQuad90deg(t, clockwise);
    }

    return t;
}

/**
 *
 */
float distance(cv::Point a, cv::Point b)
{
    return cv::norm(a - b);
}

// --------------- Drawing Stuff ---------------

/**
 *
 */
void drawCornerDots(const std::vector<cv::Point> &polygon, cv::Mat &image)
{
    for (size_t i = 0; i < polygon.size(); i++)
    {
        if (i == 0)
        {
            cv::circle(image, polygon[i], 2, cv::Scalar(255, 0, 0), 2);
        }
        else if (i == 1)
        {
            cv::circle(image, polygon[i], 2, cv::Scalar(0, 255, 255), 2);
        }
        else if (i == 2)
        {
            cv::circle(image, polygon[i], 2, cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            cv::circle(image, polygon[i], 2, cv::Scalar(0, 255, 0), 2);
        }
    }
}