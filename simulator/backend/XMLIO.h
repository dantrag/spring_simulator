#ifndef XMLIO_H
#define XMLIO_H

#include <string>
#include <vector>

#include "pugixml/pugixml.hpp"

class XMLIO {
 public:
  virtual ~XMLIO() = default;

  virtual bool loadFromXMLNode(pugi::xml_node root) = 0;
  virtual bool loadFromXML(std::string xml_file) = 0;
  void saveToXML(std::string filename) const;
  std::string toXMLString() const;

 protected:
  virtual pugi::xml_document toXML() const = 0;
};

class TypeReadable {
  public:
   bool checkType(std::string name);

  protected:
   virtual std::vector<std::string> compatible_names() = 0;
};

//bool loadFromXMLFileOrString(pugi::xml_document& xml, std::string xml_file);

#endif // XMLIO_H
