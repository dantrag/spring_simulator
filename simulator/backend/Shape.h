#ifndef SHAPE_H
#define SHAPE_H

#include <vector>

#include "pugixml/pugixml.hpp"
#include "backend/Particle.h"

class Shape {
 public:
  Shape() {}
  Shape(const Shape&) {}
  virtual ~Shape() {}
  Shape* clone() const { return nullptr; }

  virtual bool contains(const Point& point) const = 0;

  virtual double perimeter() const = 0;
  virtual double diameter() const = 0;
  virtual double area(bool oriented = false) const = 0;
  virtual Point centroid() const = 0;

  virtual std::pair<Point, Point> bounding_rectangle() const = 0;

  virtual void moveTo(const Point& new_center) = 0;
  virtual void rotateBy(double angle) = 0;
  virtual void scaleBy(double scale) = 0;
  void scaleTo(double new_area);

  virtual double distanceTo(const Point& point) const = 0;

  virtual void saveToFile(std::string filename) const = 0;
  virtual void toXML(pugi::xml_node& root_node) const = 0;
};

class Polygon : public Shape {
 public:
  Polygon(const std::vector<Point>& points);
  Polygon(const Polygon& shape);
  Polygon& operator=(const Polygon& other);
  Polygon(std::string filename);
  Polygon* clone() const { return new Polygon(points_); }
  ~Polygon() {}

  const std::vector<Point> points() const { return points_; }
  int n() const { return n_; }
  bool clockwise() const;

  bool contains(const Point& point) const override;
  bool contains_ray_casting(const Point& point) const;

  double perimeter() const override;
  double diameter() const override;
  double area(bool oriented = false) const override;
  Point centroid() const override;

  std::pair<Point, Point> bounding_rectangle() const override;

  void moveTo(const Point& new_center) override;
  void scaleBy(double scale) override;
  void rotateBy(double angle) override;

  double distanceTo(const Point& point) const override;
  double distanceTo(const Polygon& other, int samples = 100, bool hausdorff = false) const;

  Point sampleBoundary() const;
  Point sampleBoundary(double fraction) const;
  Polygon samplingContour(const std::vector<std::pair<double, double>>& vectors,
                         double margin) const;

  void saveToFile(std::string filename) const override;
  void toXML(pugi::xml_node& root_node) const override;

 protected:
  std::vector<Point> points_;
  int n_;
};


class Circle : public Shape {
 public:
  Circle(const Point& center, double radius) : center_(center), radius_(radius) {}
  Circle(std::string filename);
  Circle& operator=(const Circle& other);
  Circle* clone() { return new Circle(center_, radius_); }
  ~Circle() {}

  bool contains(const Point& point) const override { return distance2(point, center_) <= radius_ * radius_; }

  double perimeter() const override { return 3.14159265358979 * 2 * radius_; }
  double diameter() const override { return 2 * radius_; }
  double area(bool) const override { return 3.14159265358979 * radius_ * radius_; }
  Point centroid() const override { return center_; }

  std::pair<Point, Point> bounding_rectangle() const override;

  void moveTo(const Point& new_center) override { center_ = new_center; }
  void scaleBy(double scale) override { radius_ *= scale; }
  void rotateBy(double) override {}

  double distanceTo(const Point& point) const override { return abs(distance(point, center_) - radius_); }

  void saveToFile(std::string filename) const override;
  void toXML(pugi::xml_node& root_node) const override;

  double radius() const { return radius_; }
  void setRadius(double radius) { radius_ = radius; }

 private:
  Point center_ = Point(0.0, 0.0);
  double radius_ = 0.0;
};

#endif // SHAPE_H
