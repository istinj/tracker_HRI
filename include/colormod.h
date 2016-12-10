/*
 * colormod.h
 *
 *  Created on: 10/dic/2016
 *      Author: istin
 */

#ifndef COLORMOD_H_
#define COLORMOD_H_

// ***************************************************************** //
// ***************************************************************** //
// ***************************************************************** //
//#include <ostream>
//namespace Color
//{
//    enum Code
//    {
//        FG_RED      = 31,
//        FG_GREEN    = 32,
//        FG_BLUE     = 34,
//        FG_YELLOW 	= 33,
//        FG_CYAN 	= 36,
//        FG_DEFAULT  = 39,
//        BG_RED      = 41,
//        BG_GREEN    = 42,
//        BG_BLUE     = 44,
//        BG_DEFAULT  = 49
//    };
//    class Modifier
//    {
//        Code code;
//    public:
//        Modifier(Code pCode) : code(pCode) {}
//
//        friend std::ostream&
//        operator<<(std::ostream& os, const Modifier& mod)
//        {
//            return os << "\033[" << mod.code << "m";
//        }
//    };
//}
//! USE
//    Color::Modifier red(Color::FG_RED);
//    Color::Modifier def(Color::FG_DEFAULT);
//    Color::Modifier yellow(Color::FG_YELLOW);
//    Color::Modifier cyan(Color::FG_CYAN);
// ***************************************************************** //
// ***************************************************************** //
// ***************************************************************** //

#define RESET   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */



#endif /* COLORMOD_H_ */
