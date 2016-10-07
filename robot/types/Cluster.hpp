#pragma once

#include "types/Point.hpp"
#include "utils/basic_maths.hpp"
#include <iostream>

struct Cluster
{
   Cluster () {centre = Point(0,0); size = 0; id = 0; tl = Point(-1,-1); br = Point(-1,-1);}
   Cluster (Point p) : centre(p), size(1), id(0), tl(p), br(p) {};
   virtual ~Cluster () {}

   void addPoint(const Point p)
   {
      // Update the edge coordinates
      if (tl.x() == -1 || p.x() < tl.x()) {
         tl.x() = p.x();
      }
      if (br.x() == -1 || p.x() > br.x()) {
         br.x() = p.x();
      }

      if (tl.y() == -1 || p.y() < tl.y()) {
         tl.y() = p.y();
      }
      if (br.y() == -1 || p.y() > br.y()) {
         br.y() = p.y();
      }
      
      
      if (size == 0) {
         centre = p;
         size ++;
      } else {
         // Update the centre
         centre *= size;
         centre += p;
         size ++;
         centre /= size;
      }

      // Update the edge coordinates
      /*if (tl.x() == -1 || p.x() < tl.x()) {
         tl.x() = p.x();
      }
      if (br.x() == -1 || p.x() > br.x()) {
         br.x() = p.x();
      }

      if (tl.y() == -1 || p.y() < tl.y()) {
         tl.y() = p.y();
      }
      if (br.y() == -1 || p.y() > br.y()) {
         br.y() = p.y();
      }*/
   }

   Point centre;
   int size;
   int id;

   // Coordinates of the edges
   Point tl;
   Point br;

   bool operator== (const Cluster &other) const
   {
      return centre == other.centre && size == other.size;
   }

   bool operator!= (const Cluster &other) const
   {
      return (centre != other.centre || size != other.size);
   }
};

