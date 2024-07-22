/*
  This is a library written for the ST25DV64KC Dynamic RFID Tag.
  SparkFun sells these at its website:
  https://www.sparkfun.com/products/

  Do you like this library? Help support open source hardware. Buy a board!

  Written by Ricardo Ramos  @ SparkFun Electronics, January 6th, 2021
  This file declares all functions used in the ST25DV64KC Dynamic RFID Tag Arduino Library.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _ST25DV64KC_NDEF_
#define _ST25DV64KC_NDEF_

#include "ST25DV64KC_Arduino_Library.h"
#include "ST25DV64KC_IO.h"
#include "ST25DV64KC_Arduino_Library_Constants.h"

class SFE_ST25DV64KC_NDEF : public SFE_ST25DV64KC
{
private:
  uint16_t _ccFileLen = 8; // Record the length of the CC File - default to 8 bytes for the ST25DV64K

public:
  // Default constructor.
  SFE_ST25DV64KC_NDEF(){};

  // Default destructor.
  ~SFE_ST25DV64KC_NDEF(){};

  // Write a 4-byte CC File to user memory (e.g. ST25DV04K)
  // Returns true if successful, otherwise false
  bool writeCCFile4Byte(uint32_t val = 0xE1403F00);
  
  // Write an 8-byte CC File to user memory (ST25DV64K)
  // Returns true if successful, otherwise false
  bool writeCCFile8Byte(uint32_t val1 = 0xE2400001, uint32_t val2 = 0x000003FF);

  // Update _ccFileLen
  void setCCFileLen(uint16_t newLen) { _ccFileLen = newLen; }
  uint16_t getCCFileLen() { return _ccFileLen; }
  
  // Write an empty NDEF WiFi Record to user memory
  // If address is not NULL, start writing at *address, otherwise start at _ccFileLen
  // Returns true if successful, otherwise false
  bool writeNDEFEmpty(uint16_t *address = NULL);

  // Write an NDEF UTF-8 Text Record to user memory
  // If address is not NULL, start writing at *address, otherwise start at _ccFileLen
  // MB = Message Begin, ME = Message End
  // Default is a single message (MB=true, ME=true)
  // To add multiple URIs:
  //   First: MB=true, ME=false
  //   Intermediate: MB=false, ME=false
  //   Last: MB=false, ME=true
  // If language is not NULL, language is copied into the Text Record, otherwise "en" is used
  // Returns true if successful, otherwise false
  bool writeNDEFText(const char *theText, uint16_t *address, bool MB = true, bool ME = true, const char *languageCode = NULL);
  bool writeNDEFText(const uint8_t *theText, uint16_t textLength, uint16_t *address, bool MB = true, bool ME = true, const char *languageCode = NULL);

  // Read an NDEF UTF-8 Text Record from memory
  // Default is to read the first Text record (recordNo = 1). Increase recordNo to read later entries
  // maxTextLen is the maximum number of chars which theText can hold
  // If language is not NULL, the Language Code will be copied into language
  // maxLanguageLen is the maximum number of chars which language can hold
  // Returns true if successful, otherwise false
  bool readNDEFText(char *theText, uint16_t maxTextLen, uint8_t recordNo = 1, char *language = NULL, uint16_t maxLanguageLen = 0);
  // Read an NDEF UTF-8 Text Record from memory
  // *textLen should be set to the maximum number of bytes which theText can hold
  // On return, *textLen contains the actual number of bytes read
  bool readNDEFText(uint8_t *theText, uint16_t *textLen, uint8_t recordNo = 1, char *language = NULL, uint16_t maxLanguageLen = 0);
};

#endif