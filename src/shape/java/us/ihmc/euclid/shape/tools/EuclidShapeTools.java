package us.ihmc.euclid.shape.tools;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeTools
{
   public static boolean isPoint3DInsideBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize, double epsilon)
   {
      if (Math.abs(query.getX()) <= 0.5 * box3DSize.getX() + epsilon)
      {
         if (Math.abs(query.getY()) <= 0.5 * box3DSize.getY() + epsilon)
         {
            return Math.abs(query.getZ()) <= 0.5 * box3DSize.getZ() + epsilon;
         }
      }
      return false;
   }

   public static double signedDistanceBetweenPoint3DAndBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize)
   {
      return evaluatePointWithBox3D(query, null, null, box3DSize);
   }

   public static boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, Vector3DReadOnly box3DSize)
   {
      return evaluatePointWithBox3D(pointToProject, projectionToPack, null, box3DSize) <= 0.0;
   }

   public static double evaluatePointWithBox3D(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack, Vector3DReadOnly box3DSize)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();

      boolean isInside = Math.abs(query.getX()) <= halfSizeX && Math.abs(query.getY()) <= halfSizeY && Math.abs(query.getZ()) <= halfSizeZ;

      if (isInside)
      {
         double dx = Math.abs(Math.abs(x) - halfSizeX);
         double dy = Math.abs(Math.abs(y) - halfSizeY);
         double dz = Math.abs(Math.abs(z) - halfSizeZ);

         if (closestPointToPack != null)
         {
            closestPointToPack.set(x, y, z);

            if (dx < dy)
            {
               if (dx < dz)
                  closestPointToPack.setX(Math.copySign(halfSizeX, x));
               else
                  closestPointToPack.setZ(Math.copySign(halfSizeZ, z));
            }
            else
            {
               if (dy < dz)
                  closestPointToPack.setY(Math.copySign(halfSizeY, y));
               else
                  closestPointToPack.setZ(Math.copySign(halfSizeZ, z));
            }
         }

         if (normalToPack != null)
         {
            normalToPack.setToZero();

            if (dx < dy)
            {
               if (dx < dz)
                  normalToPack.setX(Math.copySign(1.0, x));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
            else
            {
               if (dy < dz)
                  normalToPack.setY(Math.copySign(1.0, y));
               else
                  normalToPack.setZ(Math.copySign(1.0, z));
            }
         }

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         double xClamped = EuclidCoreTools.clamp(x, halfSizeX);
         double yClamped = EuclidCoreTools.clamp(y, halfSizeY);
         double zClamped = EuclidCoreTools.clamp(z, halfSizeZ);

         double dx = x - xClamped;
         double dy = y - yClamped;
         double dz = z - zClamped;

         double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xClamped, yClamped, zClamped);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(1.0 / distance);
         }

         return distance;
      }
   }
}
