#ifndef PRINTCURVES_CREATEFILES_H
#define PRINTCURVES_CREATEFILES_H

#include <string>

namespace printCurves {

class CurvesPostScript;

std::string epsExt();
void curvesPostScriptToEps( const CurvesPostScript&, const std::string& filepath );
void savePSToFile( const std::string& postScript, const std::string& filepath );

} // printCurves

#endif // #include
