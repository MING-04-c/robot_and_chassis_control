#include "SimpleArgParser.hpp"

#include <cstdlib>

SimpleArgParser::SimpleArgParser(std::string program_name, std::string usage_line)
    : program_name_(std::move(program_name)), usage_line_(std::move(usage_line)) {}

void SimpleArgParser::addOption(const std::string& key, const std::string& help, bool required) {
    OptionSpec spec;
    spec.key = key;
    spec.help = help;
    spec.required = required;
    specs_.push_back(spec);
    spec_map_[key] = spec;
}

void SimpleArgParser::addOptionWithDefault(const std::string& key, const std::string& help,
                                           const std::string& default_value, bool required) {
    OptionSpec spec;
    spec.key = key;
    spec.help = help;
    spec.required = required;
    spec.default_value = default_value;
    spec.has_default = true;
    specs_.push_back(spec);
    spec_map_[key] = spec;
}

bool SimpleArgParser::parse(int argc, char** argv, std::string& error) {
    values_.clear();
    help_requested_ = false;

    for (int i = 1; i < argc; ++i) {
        std::string token(argv[i]);
        if (token == "--help" || token == "-h") {
            help_requested_ = true;
            continue;
        }

        if (token.rfind("--", 0) != 0) {
            error = "Invalid argument format: " + token;
            return false;
        }

        const auto eq_pos = token.find('=');
        if (eq_pos == std::string::npos) {
            error = "Expected --key=value format: " + token;
            return false;
        }

        const std::string key = token.substr(2, eq_pos - 2);
        const std::string value = token.substr(eq_pos + 1);

        if (spec_map_.find(key) == spec_map_.end()) {
            error = "Unknown option: --" + key;
            return false;
        }
        values_[key] = value;
    }

    if (help_requested_) {
        return true;
    }

    for (const auto& spec : specs_) {
        if (values_.find(spec.key) == values_.end() && spec.has_default) {
            values_[spec.key] = spec.default_value;
        }
    }

    for (const auto& spec : specs_) {
        if (spec.required && values_.find(spec.key) == values_.end()) {
            error = "Missing required option: --" + spec.key;
            return false;
        }
    }

    return true;
}

bool SimpleArgParser::isHelpRequested() const { return help_requested_; }

void SimpleArgParser::printHelp(std::ostream& os) const {
    os << "Usage:\n";
    os << "  " << usage_line_ << "\n\n";
    os << "Options:\n";
    for (const auto& spec : specs_) {
        os << "  --" << spec.key;
        if (spec.required) {
            os << " (required)";
        }
        os << "\n      " << spec.help;
        if (spec.has_default) {
            os << " [default: " << spec.default_value << "]";
        }
        os << "\n";
    }
    os << "  --help\n      Show this help\n";
}

bool SimpleArgParser::has(const std::string& key) const { return values_.find(key) != values_.end(); }

std::string SimpleArgParser::getString(const std::string& key) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        return "";
    }
    return it->second;
}

std::string SimpleArgParser::getStringOr(const std::string& key, const std::string& fallback) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        return fallback;
    }
    return it->second;
}

int SimpleArgParser::getIntOr(const std::string& key, int fallback, bool* ok) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        if (ok) {
            *ok = true;
        }
        return fallback;
    }

    char* end = nullptr;
    const long value = std::strtol(it->second.c_str(), &end, 10);
    const bool parsed = (end && *end == '\0');
    if (ok) {
        *ok = parsed;
    }
    return parsed ? static_cast<int>(value) : fallback;
}

double SimpleArgParser::getDoubleOr(const std::string& key, double fallback, bool* ok) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        if (ok) {
            *ok = true;
        }
        return fallback;
    }

    char* end = nullptr;
    const double value = std::strtod(it->second.c_str(), &end);
    const bool parsed = (end && *end == '\0');
    if (ok) {
        *ok = parsed;
    }
    return parsed ? value : fallback;
}

bool SimpleArgParser::parseBool(const std::string& value, bool& out) {
    if (value == "1" || value == "true" || value == "TRUE") {
        out = true;
        return true;
    }
    if (value == "0" || value == "false" || value == "FALSE") {
        out = false;
        return true;
    }
    return false;
}

bool SimpleArgParser::getBoolOr(const std::string& key, bool fallback, bool* ok) const {
    const auto it = values_.find(key);
    if (it == values_.end()) {
        if (ok) {
            *ok = true;
        }
        return fallback;
    }

    bool parsed_value = fallback;
    const bool parsed = parseBool(it->second, parsed_value);
    if (ok) {
        *ok = parsed;
    }
    return parsed ? parsed_value : fallback;
}
