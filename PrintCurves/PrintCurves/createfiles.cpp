#include <createfiles.h>

#include <curvespostscript.h>

#include <fstream>
#include <iostream>
#include <sstream>

namespace printCurves {

void curvesPostScriptToEps( const CurvesPostScript& cps, const std::string& filepath )
{
    const std::string psCode = cps.epsCode();
    savePSToFile( psCode, filepath );
}

void savePSToFile( const std::string& postScript, const std::string& filepath )
{
    std::ofstream file;
    file.open( filepath );
    file << postScript;
    file.close();
}

std::string epsExt() { return "eps"; }

} // printCurves
