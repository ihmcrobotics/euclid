package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class PointShape3D implements PointShape3DBasics, GeometryObject<PointShape3D>
{
   private double x, y, z;

   public PointShape3D()
   {
      setToZero();
   }

   public PointShape3D(Tuple3DReadOnly tuple3DReadOnly)
   {
      set(tuple3DReadOnly);
   }

   @Override
   public void set(PointShape3D other)
   {
      PointShape3DBasics.super.set(other);
   }

   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof PointShape3DReadOnly)
         return PointShape3DBasics.super.equals((PointShape3DReadOnly) object);
      else
         return false;
   }

   @Override
   public boolean epsilonEquals(PointShape3D other, double epsilon)
   {
      return PointShape3DBasics.super.epsilonEquals(other, epsilon);
   }

   @Override
   public boolean geometricallyEquals(PointShape3D other, double epsilon)
   {
      return PointShape3DBasics.super.geometricallyEquals(other, epsilon);
   }

   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, x);
      bits = EuclidHashCodeTools.addToHashCode(bits, y);
      bits = EuclidHashCodeTools.addToHashCode(bits, z);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getPointShape3DString(this);
   }
}
