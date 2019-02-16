package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Triangle3DReadOnly
{
   Point3DReadOnly getA();

   Point3DReadOnly getB();

   Point3DReadOnly getC();

   default boolean containsNaN()
   {
      return getA().containsNaN() || getB().containsNaN() || getC().containsNaN();
   }

   default double getAB()
   {
      return getA().distance(getB());
   }

   default double getBC()
   {
      return getB().distance(getC());
   }

   default double getCA()
   {
      return getC().distance(getA());
   }

   default double getABSquared()
   {
      return getA().distanceSquared(getB());
   }

   default double getBCSquared()
   {
      return getB().distanceSquared(getC());
   }

   default double getCASquared()
   {
      return getC().distanceSquared(getA());
   }

   default double getArea()
   {
      return EuclidGeometryTools.triangleArea(getA(), getB(), getC());
   }

   default boolean isEquilateral(double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getABSquared(), getCASquared(), epsilon))
         return false;

      return EuclidCoreTools.epsilonEquals(getCASquared(), getBCSquared(), epsilon);
   }

   default boolean epsilonEquals(Triangle3DReadOnly other, double epsilon)
   {
      if (!getA().epsilonEquals(other.getA(), epsilon))
         return false;
      if (!getB().epsilonEquals(other.getB(), epsilon))
         return false;
      if (!getC().epsilonEquals(other.getC(), epsilon))
         return false;
      return true;
   }

   default boolean geometricallyEquals(Triangle3DReadOnly other, double epsilon)
   {
      return geometricallyEquals(other.getA(), other.getB(), other.getC(), epsilon);
   }

   default boolean geometricallyEquals(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, double epsilon)
   {
      if (getA().geometricallyEquals(a, epsilon))
      {
         if (getB().geometricallyEquals(b, epsilon))
            return getC().geometricallyEquals(c, epsilon);
         else
            return getB().geometricallyEquals(c, epsilon) && getC().geometricallyEquals(b, epsilon);
      }
      else if (getA().geometricallyEquals(b, epsilon))
      {
         if (getB().geometricallyEquals(a, epsilon))
            return getC().geometricallyEquals(c, epsilon);
         else
            return getB().geometricallyEquals(c, epsilon) && getC().geometricallyEquals(a, epsilon);
      }
      else if (getA().geometricallyEquals(c, epsilon))
      {
         if (getB().geometricallyEquals(a, epsilon))
            return getC().geometricallyEquals(b, epsilon);
         else
            return getB().geometricallyEquals(b, epsilon) && getC().geometricallyEquals(a, epsilon);
      }
      else
      {
         return false;
      }
   }

   default boolean equals(Triangle3DReadOnly other)
   {
      if (other == null)
         return false;
      else
         return getA().equals(other.getA()) && getB().equals(other.getB()) && getC().equals(other.getC());
   }
}
