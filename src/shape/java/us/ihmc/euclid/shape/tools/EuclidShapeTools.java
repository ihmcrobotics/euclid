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
      return evaluatePoint3DWithBox3D(query, null, null, box3DSize);
   }

   public static boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, Vector3DReadOnly box3DSize)
   {
      return evaluatePoint3DWithBox3D(pointToProject, projectionToPack, null, box3DSize) <= 0.0;
   }

   public static double evaluatePoint3DWithBox3D(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack, Vector3DReadOnly box3DSize)
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

   public static double evaluatePoint3DWithCylinder3D(Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack, double cylinder3DRadius, double cylinder3DHeight)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DHeight <= 0.0)
      {
         if (closestPointOnSurfaceToPack != null)
            closestPointOnSurfaceToPack.setToNaN();
         if (normalToPack != null)
            normalToPack.setToNaN();
         return Double.NaN;
      }

      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();

      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);
      double halfHeight = 0.5 * cylinder3DHeight;

      if (xyLengthSquared <= cylinder3DRadius * cylinder3DRadius)
      {
         if (z < -halfHeight)
         { // The query is directly below the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, -halfHeight);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, -1.0);
            return -(z + halfHeight);
         }

         if (z > halfHeight)
         { // The query is directly above the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, halfHeight);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, 1.0);
            return z - halfHeight;
         }

         // The query is inside the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double dz = Math.min(halfHeight - z, z + halfHeight);
         double dr = cylinder3DRadius - xyLength;

         if (dz < dr)
         {
            if (z < 0)
            { // Closer to the bottom face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, -halfHeight);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, -1.0);
               return -(z + halfHeight);
            }
            else
            { // Closer to the top face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, halfHeight);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, 1.0);
               return z - halfHeight;
            }
         }
         else
         { // Closer to the cylinder part
            if (closestPointOnSurfaceToPack != null)
            {
               double xyScale = cylinder3DRadius / xyLength;
               closestPointOnSurfaceToPack.set(x * xyScale, y * xyScale, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x, y, 0.0);
               normalToPack.scale(1.0 / xyLength);
            }
            return xyLength - cylinder3DRadius;
         }
      }
      else
      { // The projection of the query onto the xy-plane is outside of the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double xyLengthInv = 1.0 / xyLength;

         double xyClosestScale = cylinder3DRadius * xyLengthInv;
         double xClosest = x * xyClosestScale;
         double yClosest = y * xyClosestScale;
         double zClosest = z;

         if (z < -halfHeight)
            zClosest = -halfHeight;
         else if (z > halfHeight)
            zClosest = halfHeight;

         if (zClosest != z)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double dx = x - xClosest;
            double dy = y - yClosest;
            double dz = z - zClosest;

            double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(dx, dy, dz);
               normalToPack.scale(1.0 / distance);
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(x * xyLengthInv, y * xyLengthInv, 0.0);
            }

            return xyLength - cylinder3DRadius;
         }
      }
   }
}
