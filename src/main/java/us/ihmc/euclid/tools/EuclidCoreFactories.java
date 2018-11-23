package us.ihmc.euclid.tools;

import java.util.function.DoubleSupplier;

import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidCoreFactories
{
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
         public String toString()
         {
            return EuclidCoreIOTools.getTuple3DString(this);
         }
      };
   }
}
