/*
Original source code supplied René Nyffenegger, see
https://github.com/ReneNyffenegger/cpp-base64.

Modified 2020-08-29 by https://github.com/real-tintin,
to header only and removed no needed methods. Assumes C++17.
*/

#ifndef BASE64_H
#define BASE64_H

#include <string>
#include <string_view>

inline const char* base64_chars[2] = {
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789"
             "+/",

             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789"
             "-_"};

inline unsigned int pos_of_char(const unsigned char chr) {
 //
 // Return the position of chr within base64_encode()
 //

    if      (chr >= 'A' && chr <= 'Z') return chr - 'A';
    else if (chr >= 'a' && chr <= 'z') return chr - 'a' + ('Z' - 'A')               + 1;
    else if (chr >= '0' && chr <= '9') return chr - '0' + ('Z' - 'A') + ('z' - 'a') + 2;
    else if (chr == '+' || chr == '-') return 62; // Be liberal with input and accept both url ('-') and non-url ('+') base 64 characters (
    else if (chr == '/' || chr == '_') return 63; // Ditto for '/' and '_'

    throw "If input is correct, this line should never be reached.";
}

inline std::string base64_encode(const char* bytes_to_encode, size_t in_len, bool url = false) {

    size_t len_encoded = (in_len +2) / 3 * 4;

    unsigned char trailing_char = url ? '.' : '=';

 //
 // Choose set of base64 characters. They differ
 // for the last two positions, depending on the url
 // parameter.
 // A bool (as is the parameter url) is guaranteed
 // to evaluate to either 0 or 1 in C++ therfore,
 // the correct character set is chosen by subscripting
 // base64_chars with url.
 //
    const char* base64_chars_ = base64_chars[url];

    std::string ret;
    ret.reserve(len_encoded);

    unsigned int pos = 0;

    while (pos < in_len) {
        ret.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0xfc) >> 2]);

        if (pos+1 < in_len) {
           ret.push_back(base64_chars_[((bytes_to_encode[pos + 0] & 0x03) << 4) + ((bytes_to_encode[pos + 1] & 0xf0) >> 4)]);

           if (pos+2 < in_len) {
              ret.push_back(base64_chars_[((bytes_to_encode[pos + 1] & 0x0f) << 2) + ((bytes_to_encode[pos + 2] & 0xc0) >> 6)]);
              ret.push_back(base64_chars_[  bytes_to_encode[pos + 2] & 0x3f]);
           }
           else {
              ret.push_back(base64_chars_[(bytes_to_encode[pos + 1] & 0x0f) << 2]);
              ret.push_back(trailing_char);
           }
        }
        else {

            ret.push_back(base64_chars_[(bytes_to_encode[pos + 0] & 0x03) << 4]);
            ret.push_back(trailing_char);
            ret.push_back(trailing_char);
        }

        pos += 3;
    }


    return ret;
}

inline std::string base64_decode(std::string encoded_string, bool remove_linebreaks = false) {
 //
 // decode(…) is templated so that it can be used with String = const std::string&
 // or std::string_view (requires at least C++17)
 //
    if (remove_linebreaks) {

       if (! encoded_string.length() ) {
           return "";
       }

       std::string copy(encoded_string);

       size_t pos=0;
       while ((pos = copy.find("\n", pos)) != std::string::npos) {
           copy.erase(pos, 1);
       }

       return base64_decode(copy, false);

    }

    size_t length_of_string = encoded_string.length();
    if (!length_of_string) return std::string("");

    size_t in_len = length_of_string;
    size_t pos = 0;

 //
 // The approximate length (bytes) of the decoded string might be one ore
 // two bytes smaller, depending on the amount of trailing equal signs
 // in the encoded string. This approximation is needed to reserve
 // enough space in the string to be returned.
 //
    size_t approx_length_of_decoded_string = length_of_string / 4 * 3;
    std::string ret;
    ret.reserve(approx_length_of_decoded_string);

    while (pos < in_len) {

       unsigned int pos_of_char_1 = pos_of_char(encoded_string[pos+1] );

       ret.push_back(static_cast<std::string::value_type>( ( (pos_of_char(encoded_string[pos+0]) ) << 2 ) + ( (pos_of_char_1 & 0x30 ) >> 4)));

       if (encoded_string[pos+2] != '=' && encoded_string[pos+2] != '.') { // accept URL-safe base 64 strings, too, so check for '.' also.

          unsigned int pos_of_char_2 = pos_of_char(encoded_string[pos+2] );
          ret.push_back(static_cast<std::string::value_type>( (( pos_of_char_1 & 0x0f) << 4) + (( pos_of_char_2 & 0x3c) >> 2)));

          if (encoded_string[pos+3] != '=' && encoded_string[pos+3] != '.') {
             ret.push_back(static_cast<std::string::value_type>( ( (pos_of_char_2 & 0x03 ) << 6 ) + pos_of_char(encoded_string[pos+3])   ));
          }
       }

       pos += 4;
    }

    return ret;
}

#endif /* BASE64_H */
