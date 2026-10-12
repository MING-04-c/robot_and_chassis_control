#pragma once

#include <map>
#include <ostream>
#include <string>
#include <vector>

class SimpleArgParser {
   public:
    struct OptionSpec {
        std::string key;
        std::string help;
        bool required = false;
        std::string default_value;
        bool has_default = false;
    };

    explicit SimpleArgParser(std::string program_name, std::string usage_line);

    void addOption(const std::string& key, const std::string& help, bool required = false);
    void addOptionWithDefault(const std::string& key, const std::string& help, const std::string& default_value,
                              bool required = false);

    bool parse(int argc, char** argv, std::string& error);
    bool isHelpRequested() const;
    void printHelp(std::ostream& os) const;

    bool has(const std::string& key) const;
    std::string getString(const std::string& key) const;
    std::string getStringOr(const std::string& key, const std::string& fallback) const;
    int getIntOr(const std::string& key, int fallback, bool* ok = nullptr) const;
    double getDoubleOr(const std::string& key, double fallback, bool* ok = nullptr) const;
    bool getBoolOr(const std::string& key, bool fallback, bool* ok = nullptr) const;

   private:
    static bool parseBool(const std::string& value, bool& out);

    std::string program_name_;
    std::string usage_line_;
    bool help_requested_ = false;

    std::vector<OptionSpec> specs_;
    std::map<std::string, OptionSpec> spec_map_;
    std::map<std::string, std::string> values_;
};
