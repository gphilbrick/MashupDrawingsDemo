#include <PrintCurves/printcurvesutility.h>

#include <PrintCurves/curvespostscript.h>

#include <algorithm>
#include <sstream>

namespace printCurves {

std::string defaultLineWidthToken()
{
    return "defwidth";
}

std::string defaultLengthToken()
{
    return "deflength";
}

std::string defaultRadiusToken()
{
    return "defrad";
}

double defaultLineWidth( const CurvesPostScript& psGen )
{
    const core::BoundingBoxd& canvasDims = psGen.canvasBounds();
    return 0.006 * std::min( canvasDims.widthExclusive(), canvasDims.heightExclusive() );
}

double defaultRadius( const CurvesPostScript& psGen )
{
    return defaultLineWidth( psGen ) * 2.0;
}

void writeOptionalScalar( boost::optional< double > scalar, const std::string& defToken, std::stringstream& str )
{
    if( scalar.is_initialized() ) {
        str << scalar.value();
    } else {
        str << defToken;
    }
}

bool readOptionalScalar( std::istringstream& str, const std::string& defaultToken, boost::optional< double >& out )
{
    out = boost::none;
    std::string word;
    if( str >> word ) {
        if( word == defaultToken ) {
            return true;
        } else {
            std::istringstream str2( word );
            double width = 0;
            if( str2 >> width ) {
                out = width;
                return true;
            } else {
                return false;
            }
        }
    } else {
        return false;
    }
}

void writeRGB( int r, int g, int b, std::stringstream& str )
{
    const int composite = ( r << 16 )
        | ( g << 8 )
        | b;
    str << "0x" << std::hex << composite << std::dec;
}

bool readRGB( int& oRed, int& oGreen, int& oBlue, const std::string& str )
{
    std::istringstream istr( str );
    return readRGB( oRed, oGreen, oBlue, istr );
}

bool readRGB( int& oRed, int& oGreen, int& oBlue, std::istringstream& str )
{
    int composite = 0;
    str >> std::hex;
    if( str >> composite ) {
        oBlue = composite & 0xff;
        oGreen = ( composite & 0xff00 ) >> 8;
        oRed = ( composite & 0xff0000 ) >> 16;
        return true;
    } else {
        return false;
    }
}

bool stringHasOneWord( const std::string& input, std::string& storeWord )
{
    std::istringstream str( input );
    if( str >> storeWord ) {
        std::string secondWord;
        return !( str >> secondWord );
    } else {
        return false;
    }
}

size_t numWordsInString( const std::string& input )
{
    size_t sum = 0;
    std::string storeWord;
    std::istringstream str( input );
    while( str >> storeWord ) {
        sum++;
    }
    return sum;
}

} // printCurves
