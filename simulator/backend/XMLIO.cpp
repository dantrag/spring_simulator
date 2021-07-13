#include "backend/XMLIO.h"

#include <algorithm>

void XMLIO::saveToXML(std::string filename) const {
  toXML().save_file(filename.c_str());
}

std::string XMLIO::toXMLString() const {
  pugi::xml_writer_string writer;
  toXML().save(writer);
  return writer.result;
}

/*
bool loadFromXMLFileOrString(pugi::xml_document& xml, std::string xml_file) {
  auto extension = xml_file.substr(xml_file.find_last_of(".") + 1, std::string::npos);
  std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

  auto status = pugi::xml_parse_status::status_file_not_found;
  if (extension == "xml")
    status = xml.load_file(xml_file.c_str()).status;
  else
    status = xml.load_string(xml_file.c_str()).status;
  return status == pugi::xml_parse_status::status_ok;
}
*/

bool TypeReadable::checkType(std::string name) {
  std::transform(name.begin(), name.end(), name.begin(), ::tolower);
  for (const auto& allowed_name : compatible_names()) {
    if (name == allowed_name) return true;
  }
  return false;
}
