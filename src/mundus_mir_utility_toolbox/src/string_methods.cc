#include "utility_toolbox/string_methods.h"


// Function that takes in a string and a char, and returns a vector with substrings split at the given char
vector<string> utility_toolbox::split_string(const string &s, char delimiter) {

    // Extracts line by line decided by delimiter 
    vector<string> tokens;
    string token;
    istringstream token_stream(s);
    while (getline(token_stream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}