package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Triangle3D
{
   private final Point3D a = new Point3D();
   private final Point3D b = new Point3D();
   private final Point3D c = new Point3D();

   public Triangle3D()
   {
   }

   public Triangle3D(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      set(a, b, c);
   }

   public Triangle3D(Triangle3D other)
   {
      set(other);
   }

   public void set(Triangle3D other)
   {
      set(other.getA(), other.getB(), other.getC());
   }

   public void set(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      this.a.set(a);
      this.b.set(b);
      this.c.set(c);
   }

   public Point3D getA()
   {
      return a;
   }

   public Point3D getB()
   {
      return b;
   }

   public Point3D getC()
   {
      return c;
   }

   public double getAB()
   {
      return a.distance(b);
   }

   public double getBC()
   {
      return b.distance(c);
   }

   public double getCA()
   {
      return c.distance(a);
   }

   public double getArea()
   {
      return EuclidGeometryTools.triangleArea(a, b, c);
   }

   public boolean isEquilateral(double epsilon)
   {
      double abSquare = a.distanceSquared(b);
      double acSquare = a.distanceSquared(c);

      if (!EuclidCoreTools.epsilonEquals(abSquare, acSquare, epsilon))
         return false;

      double bcSquare = b.distanceSquared(c);

      return EuclidCoreTools.epsilonEquals(acSquare, bcSquare, epsilon);
   }

   public boolean geometryEquals(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, double epsilon)
   {
      if (this.a.geometricallyEquals(a, epsilon))
      {
         if (this.b.geometricallyEquals(b, epsilon))
            return this.c.geometricallyEquals(c, epsilon);
         else
            return this.b.geometricallyEquals(c, epsilon) && this.c.geometricallyEquals(b, epsilon);
      }
      else if (this.a.geometricallyEquals(b, epsilon))
      {
         if (this.b.geometricallyEquals(a, epsilon))
            return this.c.geometricallyEquals(c, epsilon);
         else
            return this.b.geometricallyEquals(c, epsilon) && this.c.geometricallyEquals(a, epsilon);
      }
      else if (this.a.geometricallyEquals(c, epsilon))
      {
         if (this.b.geometricallyEquals(a, epsilon))
            return this.c.geometricallyEquals(b, epsilon);
         else
            return this.b.geometricallyEquals(b, epsilon) && this.c.geometricallyEquals(a, epsilon);
      }
      else
      {
         return false;
      }
   }
}