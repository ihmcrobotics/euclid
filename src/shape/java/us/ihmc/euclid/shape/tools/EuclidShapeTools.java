package us.ihmc.euclid.shape.tools;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
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

   public static boolean orthogonalProjectionOntoBox3D(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, Vector3DReadOnly box3DSize)
   {
      return evaluatePoint3DWithBox3D(pointToProject, projectionToPack, null, box3DSize) <= 0.0;
   }

   public static double evaluatePoint3DWithBox3D(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack,
                                                 Vector3DReadOnly box3DSize)
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

   public static boolean isPoint3DInsideCylinder3D(Point3DReadOnly query, double epsilon, double cylinder3DRadius, double cylinder3DHeight)
   {
      double halfHeightPlusEpsilon = 0.5 * cylinder3DHeight + epsilon;
      if (query.getZ() < -halfHeightPlusEpsilon || query.getZ() > halfHeightPlusEpsilon)
         return false;

      double radiusWithEpsilon = cylinder3DRadius + epsilon;
      return EuclidCoreTools.normSquared(query.getX(), query.getY()) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndCylinder3D(Point3DReadOnly query, double cylinder3DRadius, double cylinder3DHeight)
   {
      return evaluatePoint3DWithCylinder3D(query, null, null, cylinder3DRadius, cylinder3DHeight);
   }

   public static boolean orthogonalProjectionOntoCylinder3D(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, double cylinder3DRadius,
                                                            double cylinder3DHeight)
   {
      return evaluatePoint3DWithCylinder3D(pointToProject, projectionToPack, null, cylinder3DRadius, cylinder3DHeight) <= 0.0;
   }

   public static double evaluatePoint3DWithCylinder3D(Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack,
                                                      double cylinder3DRadius, double cylinder3DHeight)
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

   public static boolean isPoint3DInsideEllipsoid3D(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii, double epsilon)
   {
      double scaledX = query.getX() / (ellipsoid3DRadii.getX() + epsilon);
      double scaledY = query.getY() / (ellipsoid3DRadii.getY() + epsilon);
      double scaledZ = query.getZ() / (ellipsoid3DRadii.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   public static double signedDistanceBetweenPoint3DAndEllipsoid3D(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii)
   {
      return evaluatePoint3DWithEllipsoid3D(query, null, null, ellipsoid3DRadii);
   }

   public static boolean orthogonalProjectionOntoEllipsoid3D(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, Vector3DReadOnly ellipsoid3DRadii)
   {
      return evaluatePoint3DWithEllipsoid3D(pointToProject, projectionToPack, null, ellipsoid3DRadii) <= 0.0;
   }

   public static double evaluatePoint3DWithEllipsoid3D(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack,
                                                       Vector3DReadOnly ellipsoid3DRadii)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double sumOfSquares = EuclidCoreTools.normSquared(query.getX() / xRadius, query.getY() / yRadius, query.getZ() / zRadius);
      double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

      if (sumOfSquares > 1.0e-10)
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(query);
            closestPointToPack.scale(scaleFactor);
         }

         if (normalToPack != null)
         {
            double xScale = 1.0 / (xRadius * xRadius);
            double yScale = 1.0 / (yRadius * yRadius);
            double zScale = 1.0 / (zRadius * zRadius);

            normalToPack.set(query);
            normalToPack.scale(xScale, yScale, zScale);
            normalToPack.normalize();
         }

         return query.distanceFromOrigin() * (1.0 - scaleFactor);
      }
      else
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.set(0.0, 0.0, zRadius);
         }

         if (normalToPack != null)
         {
            normalToPack.set(0.0, 0.0, 1.0);
         }

         return query.getZ() - zRadius;
      }
   }

   public static double computeRamp3DLength(Vector3DReadOnly ramp3DSize)
   {
      return Math.sqrt(EuclidCoreTools.normSquared(ramp3DSize.getX(), ramp3DSize.getZ()));
   }

   public static double computeRanp3DIncline(Vector3DReadOnly ramp3DSize)
   {
      return Math.atan(ramp3DSize.getZ() / ramp3DSize.getX());
   }

   public static boolean isPoint3DInsideRamp3D(Point3DReadOnly query, double epsilon, Vector3DReadOnly ramp3DSize)
   {
      if (query.getZ() < -epsilon)
         return false;

      if (query.getX() > ramp3DSize.getX() + epsilon)
         return false;

      double halfWidth = 0.5 * ramp3DSize.getY() + epsilon;

      if (query.getY() < -halfWidth || query.getY() > halfWidth)
         return false;

      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;

      // Computing the signed distance between the query and the slope face, negative value means the query is below the slope.
      if (rampDirectionX * query.getZ() - query.getX() * rampDirectionZ > epsilon)
         return false;

      return true;
   }

   public static double signedDistanceBetweenPoint3DAndRamp3D(Point3DReadOnly query, Vector3DReadOnly ramp3DSize)
   {
      return evaluatePoint3DWithRamp3D(query, null, null, ramp3DSize);
   }

   public static boolean orthogonalProjectionOntoRamp3D(Point3DReadOnly pointToProject, Point3DBasics projectionToPack, Vector3DReadOnly ramp3DSize)
   {
      return evaluatePoint3DWithRamp3D(pointToProject, projectionToPack, null, ramp3DSize) <= 0.0;
   }

   public static double evaluatePoint3DWithRamp3D(Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack,
                                                  Vector3DReadOnly ramp3DSize)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();

      double halfWidth = 0.5 * ramp3DSize.getY();
      if (z < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(x, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * ramp3DSize.getY(), halfWidth);
         double zClosest = 0.0;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         }

         if (xClosest == x && yClosest == y)
         {
            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -z;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (x > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * ramp3DSize.getY(), halfWidth);
         double zClosest = EuclidCoreTools.clamp(z, 0.0, ramp3DSize.getZ());

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         }

         if (yClosest == y && zClosest == z)
         {
            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return x - ramp3DSize.getX();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(y, -0.5 * ramp3DSize.getY(), halfWidth);
         double zClosest = 0.0;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         }

         return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(y) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, y);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = x;
            double zClosest = z;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, Math.copySign(1.0, y), 0.0);
            }

            return Math.abs(y) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = x * rampDirectionX + z * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            return computeNormalAndDistanceFromClosestPoint(x, y, z, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(x, z, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = x * rampDirectionX + z * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = y;
         double zClosest = dot * rampDirectionZ;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         }

         if (normalToPack != null)
         {
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
         }

         return rampDirectionX * z - x * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - y);
         double distanceToLeftFace = halfWidth - y;
         double distanceToRearFace = ramp3DSize.getX() - x;
         double distanceToBottomFace = z;
         double distanceToSlopeFace = -(rampDirectionX * z - x * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(x, -halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, -1.0, 0.0);
            }

            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(x, halfWidth, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 1.0, 0.0);
            }

            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(ramp3DSize.getX(), y, z);
            }

            if (normalToPack != null)
            {
               normalToPack.set(1.0, 0.0, 0.0);
            }

            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(x, y, 0.0);
            }

            if (normalToPack != null)
            {
               normalToPack.set(0.0, 0.0, -1.0);
            }

            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            if (closestPointOnSurfaceToPack != null)
            {
               double dot = x * rampDirectionX + z * rampDirectionZ;
               double xClosest = dot * rampDirectionX;
               double yClosest = y;
               double zClosest = dot * rampDirectionZ;

               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            }

            if (normalToPack != null)
            {
               normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            }

            return -distanceToSlopeFace;
         }
      }
   }

   private static double computeNormalAndDistanceFromClosestPoint(double x, double y, double z, double xClosest, double yClosest, double zClosest,
                                                                  Vector3DBasics normalToPack)
   {
      double dx = x - xClosest;
      double dy = y - yClosest;
      double dz = z - zClosest;

      double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

      if (normalToPack != null)
      {
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);
      }

      return distance;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3, double value4)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3 && possibleMin <= value4;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3;
   }

   private static boolean isFirstValueMinimum(double possibleMin, double value1, double value2)
   {
      return possibleMin <= value1 && possibleMin <= value2;
   }
}
