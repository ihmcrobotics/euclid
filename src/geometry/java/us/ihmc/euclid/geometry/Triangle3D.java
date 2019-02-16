package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Triangle3DBasics;
import us.ihmc.euclid.geometry.interfaces.Triangle3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class Triangle3D implements Triangle3DBasics, GeometryObject<Triangle3D>
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

   public Triangle3D(Triangle3DReadOnly other)
   {
      set(other);
   }

   public void set(Triangle3D other)
   {
      Triangle3DBasics.super.set(other);
   }

   @Override
   public Point3D getA()
   {
      return a;
   }

   @Override
   public Point3D getB()
   {
      return b;
   }

   @Override
   public Point3D getC()
   {
      return c;
   }

   @Override
   public boolean epsilonEquals(Triangle3D other, double epsilon)
   {
      return Triangle3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(Triangle3D other, double epsilon)
   {
      return Triangle3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Triangle3DReadOnly)
         return Triangle3DBasics.super.equals((Triangle3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      long hashCode = EuclidHashCodeTools.combineHashCode(getA().hashCode(), getB().hashCode());
      hashCode = EuclidHashCodeTools.combineHashCode(hashCode, getC().hashCode());
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }

   @Override
   public String toString()
   {
      return EuclidGeometryIOTools.getTriangle3DString(this);
   }
}