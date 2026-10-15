#include "cbf_rrt_planner/map_loader.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace cbf_rrt_planner
{

    namespace
    {
        std::string trim(const std::string & s)
        {
            const char * ws = " \t\r\n\"'";
            size_t b = s.find_first_not_of(ws);
            if (b == std::string::npos) {return "";}
            size_t e = s.find_last_not_of(ws);
            return s.substr(b, e - b + 1);
        }

        // next header token of a PNM file, skipping '#' comments
        std::string pnmToken(std::istream & in)
        {
            std::string tok;
            while (in >> tok) {
                if (tok[0] == '#') {
                    std::string rest;
                    std::getline(in, rest);
                    continue;
                }
                return tok;
            }
            throw std::runtime_error("truncated PGM header");
        }

        struct Image
        {
            int width = 0;
            int height = 0;
            int maxval = 255;
            std::vector<int> pixels; // row-major, row 0 = top of the image
        };

        Image readPgm(const std::string & path)
        {
            std::ifstream in(path, std::ios::binary);
            if (!in) {throw std::runtime_error("cannot open map image '" + path + "'");}

            Image img;
            std::string magic = pnmToken(in);
            if (magic != "P5" && magic != "P2") {
                throw std::runtime_error("'" + path + "' is not a grayscale PGM (P5/P2) - convert it first");
            }
            img.width = std::stoi(pnmToken(in));
            img.height = std::stoi(pnmToken(in));
            img.maxval = std::stoi(pnmToken(in));
            img.pixels.resize(static_cast<size_t>(img.width) * img.height);

            if (magic == "P2") {
                for (auto & p : img.pixels) {p = std::stoi(pnmToken(in));}
                return img;
            }

            in.get(); // the single whitespace that ends the header
            const bool wide = img.maxval > 255;
            for (auto & p : img.pixels) {
                int hi = in.get();
                p = wide ? (hi << 8) | in.get() : hi;
            }
            if (!in) {throw std::runtime_error("truncated PGM data in '" + path + "'");}
            return img;
        }
    }

    std::map<std::string, std::string> readFlatYaml(const std::string & path)
    {
        std::ifstream f(path);
        if (!f) {throw std::runtime_error("cannot open '" + path + "'");}
        std::map<std::string, std::string> out;
        std::string line;
        while (std::getline(f, line)) {
            size_t hash = line.find('#');
            if (hash != std::string::npos) {line = line.substr(0, hash);}
            size_t colon = line.find(':');
            if (colon == std::string::npos) {continue;}
            std::string value = trim(line.substr(colon + 1));
            if (value.empty()) {continue;} // section headers such as 'ros__parameters:'
            out[trim(line.substr(0, colon))] = value;
        }
        return out;
    }

    OccupancyGridData loadMapYaml(const std::string & yaml_path)
    {
        auto yaml = readFlatYaml(yaml_path);
        auto get = [&](const std::string & key) {
            auto it = yaml.find(key);
            if (it == yaml.end()) {throw std::runtime_error("map yaml has no '" + key + "' key");}
            return it->second;
        };

        std::filesystem::path image_path(get("image"));
        if (image_path.is_relative()) {
            image_path = std::filesystem::path(yaml_path).parent_path() / image_path;
        }

        const double resolution = std::stod(get("resolution"));
        const double occupied_thresh = std::stod(get("occupied_thresh"));
        const double free_thresh = std::stod(get("free_thresh"));
        const bool negate = yaml.count("negate") && std::stoi(yaml["negate"]) != 0;
        const std::string mode = yaml.count("mode") ? yaml["mode"] : "trinary";

        std::string origin = get("origin");
        for (char & ch : origin) {
            if (ch == '[' || ch == ']' || ch == ',') {ch = ' ';}
        }
        std::istringstream origin_in(origin);
        double ox = 0.0, oy = 0.0;
        origin_in >> ox >> oy;

        Image img = readPgm(image_path.string());

        OccupancyGridData grid;
        grid.width = img.width;
        grid.height = img.height;
        grid.resolution = resolution;
        grid.origin_x = ox;
        grid.origin_y = oy;
        grid.data.resize(img.pixels.size());

        for (int r = 0; r < img.height; ++r) {
            for (int c = 0; c < img.width; ++c) {
                int v = img.pixels[static_cast<size_t>(r) * img.width + c];
                int8_t cell;
                if (mode == "raw") {
                    cell = v <= 100 ? static_cast<int8_t>(v) : static_cast<int8_t>(-1);
                } else {
                    double shade = static_cast<double>(v) / img.maxval;
                    double occ = negate ? shade : 1.0 - shade;
                    if (occ > occupied_thresh) {
                        cell = 100;
                    } else if (occ < free_thresh) {
                        cell = 0;
                    } else if (mode == "scale") {
                        cell = static_cast<int8_t>(
                            std::rint((occ - free_thresh) / (occupied_thresh - free_thresh) * 100.0));
                    } else {
                        cell = -1;
                    }
                }
                grid.data[static_cast<size_t>(img.height - 1 - r) * img.width + c] = cell;
            }
        }
        return grid;
    }

}
