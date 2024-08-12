#ifndef PRINTCURVES_PRINTCURVESUTILITY_H
#define PRINTCURVES_PRINTCURVESUTILITY_H

#include <boost/optional.hpp>

#include <memory>
#include <string>

namespace printCurves {

class CurvesPostScript;

std::string defaultLineWidthToken();
std::string defaultRadiusToken();
std::string defaultLengthToken();
double defaultLineWidth( const CurvesPostScript& psGen );
double defaultRadius( const CurvesPostScript& psGen );

/// Write either the value or the default token.
void writeOptionalScalar( boost::optional< double >, const std::string& defToken, std::stringstream& str );
/// Read either a number representing line width or the token indicating a default value.
bool readOptionalScalar( std::istringstream& str, const std::string& defaultToken, boost::optional< double >& out );

void writeRGB( int r, int g, int b, std::stringstream& str );
bool readRGB( int& oRed, int& oGreen, int& oBlue, std::istringstream& str );
bool readRGB( int& oRed, int& oGreen, int& oBlue, const std::string& str );

/// Return whether 'input' contains only one word, and store this word in 'storeWord' if this is the case.
bool stringHasOneWord( const std::string& input, std::string& storeWord );
size_t numWordsInString( const std::string& input );

} // printCurves

#endif // #include
