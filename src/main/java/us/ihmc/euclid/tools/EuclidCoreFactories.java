package us.ihmc.euclid.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidCoreFactories
{
   public static Vector3DReadOnly newNegativeLinkedVector3D(Vector3DReadOnly originalVector)
   {
      return new Vector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return -originalVector.getX();
         }

         @Override
         public double getY()
         {
            return -originalVector.getY();
         }

         @Override
         public double getZ()
         {
            return -originalVector.getZ();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Tuple3DReadOnly)
               return equals((Tuple3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   public static Vector3DReadOnly newLinkedVector3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return new Vector3DReadOnly()
      {
         @Override
         public double getX()
         {
            return xSupplier.getAsDouble();
         }

         @Override
         public double getY()
         {
            return ySupplier.getAsDouble();
         }

         @Override
         public double getZ()
         {
            return zSupplier.getAsDouble();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Vector3DReadOnly)
               return Vector3DReadOnly.super.equals((Vector3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }

   public static Point3DReadOnly newLinkedPoint3DReadOnly(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier)
   {
      return new Point3DReadOnly()
      {
         @Override
         public double getX()
         {
            return xSupplier.getAsDouble();
         }

         @Override
         public double getY()
         {
            return ySupplier.getAsDouble();
         }

         @Override
         public double getZ()
         {
            return zSupplier.getAsDouble();
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof Point3DReadOnly)
               return Point3DReadOnly.super.equals((Point3DReadOnly) object);
            else
               return false;
         }

         @Override
         public int hashCode()
         {
            long bits = 1L;
            bits = EuclidHashCodeTools.addToHashCode(bits, getX());
            bits = EuclidHashCodeTools.addToHashCode(bits, getY());
            bits = EuclidHashCodeTools.addToHashCode(bits, getZ());
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }
}
