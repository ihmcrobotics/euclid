package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeTools
{
   private static final double SPHERE_SMALLEST_DISTANCE_TO_ORIGIN = 1.0e-12;

   public static boolean isPoint3DInsideBox3D(Vector3DReadOnly box3DSize, Point3DReadOnly query, double epsilon)
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

   public static double signedDistanceBetweenPoint3DAndBox3D(Vector3DReadOnly box3DSize, Point3DReadOnly query)
   {
      return doPoint3DBox3DCollisionTest(box3DSize, query, null, null);
   }

   public static boolean orthogonalProjectionOntoBox3D(Vector3DReadOnly box3DSize, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return doPoint3DBox3DCollisionTest(box3DSize, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DBox3DCollisionTest(Vector3DReadOnly box3DSize, Point3DReadOnly query, Point3DBasics closestPointToPack,
                                                    Vector3DBasics normalToPack)
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

   public static boolean isPoint3DInsideCylinder3D(double cylinder3DLength, double cylinder3DRadius, Point3DReadOnly query, double epsilon)
   {
      double halfLengthPlusEpsilon = 0.5 * cylinder3DLength + epsilon;
      if (query.getZ() < -halfLengthPlusEpsilon || query.getZ() > halfLengthPlusEpsilon)
         return false;

      double radiusWithEpsilon = cylinder3DRadius + epsilon;
      return EuclidCoreTools.normSquared(query.getX(), query.getY()) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndCylinder3D(double cylinder3DLength, double cylinder3DRadius, Point3DReadOnly query)
   {
      return doPoint3DCylinder3DCollisionTest(cylinder3DLength, cylinder3DRadius, query, null, null);
   }

   public static boolean orthogonalProjectionOntoCylinder3D(double cylinder3DLength, double cylinder3DRadius, Point3DReadOnly pointToProject,
                                                            Point3DBasics projectionToPack)
   {
      return doPoint3DCylinder3DCollisionTest(cylinder3DLength, cylinder3DRadius, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DCylinder3DCollisionTest(double cylinder3DLength, double cylinder3DRadius, Point3DReadOnly query,
                                                         Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
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
      double halfLength = 0.5 * cylinder3DLength;

      if (xyLengthSquared <= cylinder3DRadius * cylinder3DRadius)
      {
         if (z < -halfLength)
         { // The query is directly below the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, -halfLength);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, -1.0);
            return -(z + halfLength);
         }

         if (z > halfLength)
         { // The query is directly above the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(x, y, halfLength);
            if (normalToPack != null)
               normalToPack.set(0.0, 0.0, 1.0);
            return z - halfLength;
         }

         // The query is inside the cylinder
         double xyLength = Math.sqrt(xyLengthSquared);
         double dz = Math.min(halfLength - z, z + halfLength);
         double dr = cylinder3DRadius - xyLength;

         if (dz < dr)
         {
            if (z < 0)
            { // Closer to the bottom face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, -halfLength);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, -1.0);
               return -(z + halfLength);
            }
            else
            { // Closer to the top face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.set(x, y, halfLength);
               if (normalToPack != null)
                  normalToPack.set(0.0, 0.0, 1.0);
               return z - halfLength;
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

         if (z < -halfLength)
            zClosest = -halfLength;
         else if (z > halfLength)
            zClosest = halfLength;

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

   public static boolean isPoint3DInsideEllipsoid3D(Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly query, double epsilon)
   {
      double scaledX = query.getX() / (ellipsoid3DRadii.getX() + epsilon);
      double scaledY = query.getY() / (ellipsoid3DRadii.getY() + epsilon);
      double scaledZ = query.getZ() / (ellipsoid3DRadii.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   public static double signedDistanceBetweenPoint3DAndEllipsoid3D(Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly query)
   {
      return doPoint3DEllipsoid3DCollisionTest(ellipsoid3DRadii, query, null, null);
   }

   public static boolean orthogonalProjectionOntoEllipsoid3D(Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return doPoint3DEllipsoid3DCollisionTest(ellipsoid3DRadii, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DEllipsoid3DCollisionTest(Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly query, Point3DBasics closestPointToPack,
                                                          Vector3DBasics normalToPack)
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
      return computeRamp3DLength(ramp3DSize.getX(), ramp3DSize.getZ());
   }

   public static double computeRamp3DLength(double ramp3DSizeX, double ramp3DSizeZ)
   {
      return Math.sqrt(EuclidCoreTools.normSquared(ramp3DSizeX, ramp3DSizeZ));
   }

   public static double computeRanp3DIncline(Vector3DReadOnly ramp3DSize)
   {
      return computeRanp3DIncline(ramp3DSize.getX(), ramp3DSize.getZ());
   }

   public static double computeRanp3DIncline(double ramp3DSizeX, double ramp3DSizeZ)
   {
      return Math.atan(ramp3DSizeZ / ramp3DSizeX);
   }

   public static boolean isPoint3DInsideRamp3D(Vector3DReadOnly ramp3DSize, Point3DReadOnly query, double epsilon)
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

   public static double signedDistanceBetweenPoint3DAndRamp3D(Vector3DReadOnly ramp3DSize, Point3DReadOnly query)
   {
      return doPoint3DRamp3DCollisionTest(ramp3DSize, query, null, null);
   }

   public static boolean orthogonalProjectionOntoRamp3D(Vector3DReadOnly ramp3DSize, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return doPoint3DRamp3DCollisionTest(ramp3DSize, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DRamp3DCollisionTest(Vector3DReadOnly ramp3DSize, Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack,
                                                     Vector3DBasics normalToPack)
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

   public static boolean isPoint3DInsideSphere3D(double sphereRadius, Tuple3DReadOnly spherePosition, Point3DReadOnly query, double epsilon)
   {
      double radiusWithEpsilon = sphereRadius + epsilon;
      double distanceSquared = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(spherePosition.getX(), spherePosition.getY(), spherePosition.getZ(), query);
      return distanceSquared <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndSphere3D(double sphereRadius, Tuple3DReadOnly spherePosition, Point3DReadOnly query)
   {
      return doPoint3DSphere3DCollisionTest(sphereRadius, spherePosition, query, null, null);
   }

   public static boolean orthogonalProjectionOntoSphere3D(double sphereRadius, Tuple3DReadOnly spherePosition, Point3DReadOnly pointToProject,
                                                          Point3DBasics projectionToPack)
   {
      return doPoint3DSphere3DCollisionTest(sphereRadius, spherePosition, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DSphere3DCollisionTest(double sphereRadius, Tuple3DReadOnly spherePosition, Point3DReadOnly query,
                                                       Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double distance = EuclidGeometryTools.distanceBetweenPoint3Ds(spherePosition.getX(), spherePosition.getY(), spherePosition.getZ(), query);

      if (closestPointToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            closestPointToPack.sub(query, spherePosition);
            closestPointToPack.scale(sphereRadius / distance);
         }
         else
         {
            closestPointToPack.set(0.0, 0.0, sphereRadius);
         }
         closestPointToPack.add(spherePosition);
      }

      if (normalToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            normalToPack.sub(query, spherePosition);
            normalToPack.scale(1.0 / distance);
         }
         else
         {
            normalToPack.set(0.0, 0.0, 1.0);
         }
      }

      return distance - sphereRadius;
   }

   public static double doSphere3DSphere3DCollisionTest(double firstSphereRadius, Tuple3DReadOnly firstSpherePosition, double secondSphereRadius,
                                                        Tuple3DReadOnly secondSpherePosition, Point3DBasics firstClosestPointToPack,
                                                        Point3DBasics secondClosestPointToPack, Vector3DBasics firstNormalToPack,
                                                        Vector3DBasics secondNormalToPack)
   {
      double distance = EuclidGeometryTools.distanceBetweenPoint3Ds(firstSpherePosition.getX(), firstSpherePosition.getY(), firstSpherePosition.getZ(),
                                                                    secondSpherePosition.getX(), secondSpherePosition.getY(), secondSpherePosition.getZ());

      if (firstClosestPointToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            firstClosestPointToPack.sub(secondSpherePosition, firstSpherePosition);
            firstClosestPointToPack.scale(firstSphereRadius / distance);
         }
         else
         {
            firstClosestPointToPack.set(0.0, 0.0, firstSphereRadius);
         }
         firstClosestPointToPack.add(firstSpherePosition);
      }

      if (secondClosestPointToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            secondClosestPointToPack.sub(firstSpherePosition, secondSpherePosition);
            secondClosestPointToPack.scale(secondSphereRadius / distance);
         }
         else
         {
            secondClosestPointToPack.set(0.0, 0.0, secondSphereRadius);
         }
         secondClosestPointToPack.add(secondSpherePosition);
      }

      if (firstNormalToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            firstNormalToPack.sub(secondSpherePosition, firstSpherePosition);
            firstNormalToPack.scale(1.0 / distance);
         }
         else
         {
            firstNormalToPack.set(0.0, 0.0, 1.0);
         }

         if (secondNormalToPack != null)
            secondNormalToPack.setAndNegate(firstNormalToPack);
      }
      else if (secondClosestPointToPack != null)
      {
         if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
         {
            secondNormalToPack.sub(firstSpherePosition, secondSpherePosition);
            secondNormalToPack.scale(1.0 / distance);
         }
         else
         {
            secondNormalToPack.set(0.0, 0.0, 1.0);
         }
      }

      return distance - firstSphereRadius - secondSphereRadius;
   }

   public static double doSphere3DCylinder3DCollisionTest(double sphereRadius, Tuple3DReadOnly spherePosition, double cylinderRadius, double cylinderLength,
                                                          Tuple3DReadOnly cylinderPosition, Vector3DReadOnly cylinderAxis,
                                                          Point3DBasics firstClosestPointToPack, Point3DBasics secondClosestPointToPack,
                                                          Vector3DBasics firstNormalToPack, Vector3DBasics secondNormalToPack)
   {
      double cylinderHalfLength = 0.5 * cylinderLength;
      double cylinderAxisMagnitude = cylinderAxis.length();

      double dx = spherePosition.getX() - cylinderPosition.getX();
      double dy = spherePosition.getY() - cylinderPosition.getY();
      double dz = spherePosition.getZ() - cylinderPosition.getZ();

      double crossX = cylinderAxis.getY() * dz - cylinderAxis.getZ() * dy;
      double crossY = cylinderAxis.getZ() * dx - cylinderAxis.getX() * dz;
      double crossZ = cylinderAxis.getX() * dy - cylinderAxis.getY() * dx;
      double distanceSquaredFromCylinderAxis = normSquared(crossX, crossY, crossZ) / cylinderAxisMagnitude;
      double positionOnCylinderAxis = (dx * cylinderAxis.getX() + dy * cylinderAxis.getY() + dz * cylinderAxis.getZ()) / cylinderAxisMagnitude;

      if (distanceSquaredFromCylinderAxis <= cylinderRadius * cylinderRadius)
      { // The sphere's center is contained by the cylinder with infinite length.
         if (positionOnCylinderAxis < -cylinderHalfLength)
         { // The sphere's center is below the cylinder
            if (firstClosestPointToPack != null)
               firstClosestPointToPack.scaleAdd(sphereRadius, cylinderAxis, spherePosition);
            if (secondClosestPointToPack != null)
               secondClosestPointToPack.scaleAdd(-positionOnCylinderAxis - cylinderHalfLength, cylinderAxis, spherePosition);
            if (firstNormalToPack != null)
               firstNormalToPack.set(cylinderAxis);
            if (secondNormalToPack != null)
               secondNormalToPack.setAndNegate(cylinderAxis);
            return -positionOnCylinderAxis - cylinderHalfLength - sphereRadius;
         }

         if (positionOnCylinderAxis > cylinderHalfLength)
         { // The sphere's center is above the cylinder
            if (firstClosestPointToPack != null)
               firstClosestPointToPack.scaleAdd(-sphereRadius, cylinderAxis, spherePosition);
            if (secondClosestPointToPack != null)
               secondClosestPointToPack.scaleAdd(-positionOnCylinderAxis + cylinderHalfLength, cylinderAxis, spherePosition);
            if (firstNormalToPack != null)
               firstNormalToPack.setAndNegate(cylinderAxis);
            if (secondNormalToPack != null)
               secondNormalToPack.set(cylinderAxis);
            return positionOnCylinderAxis - cylinderHalfLength - sphereRadius;
         }

         // The sphere's center is inside the cylinder
         double distanceFromCylinderAxis = Math.sqrt(distanceSquaredFromCylinderAxis);
         double dh = cylinderHalfLength - Math.abs(positionOnCylinderAxis);
         double dr = cylinderRadius - distanceFromCylinderAxis;

         if (dh < dr)
         { // Closer to either top or bottom face of the cylinder
            if (positionOnCylinderAxis < 0.0)
            { // Closer to the bottom face
               if (firstClosestPointToPack != null)
                  firstClosestPointToPack.scaleAdd(sphereRadius, cylinderAxis, spherePosition);
               if (secondClosestPointToPack != null)
                  secondClosestPointToPack.scaleAdd(-positionOnCylinderAxis - cylinderHalfLength, cylinderAxis, spherePosition);
               if (firstNormalToPack != null)
                  firstNormalToPack.set(cylinderAxis);
               if (secondNormalToPack != null)
                  secondNormalToPack.setAndNegate(cylinderAxis);
               return -positionOnCylinderAxis - cylinderHalfLength - sphereRadius;
            }
            else
            { // Closer to the top face
               if (firstClosestPointToPack != null)
                  firstClosestPointToPack.scaleAdd(-sphereRadius, cylinderAxis, spherePosition);
               if (secondClosestPointToPack != null)
                  secondClosestPointToPack.scaleAdd(-positionOnCylinderAxis + cylinderHalfLength, cylinderAxis, spherePosition);
               if (firstNormalToPack != null)
                  firstNormalToPack.setAndNegate(cylinderAxis);
               if (secondNormalToPack != null)
                  secondNormalToPack.set(cylinderAxis);
               return positionOnCylinderAxis - cylinderHalfLength - sphereRadius;
            }
         }
         else
         { // Closer to the cylinder's side
            double sphereProjectionOnCylinderAxisX = positionOnCylinderAxis * cylinderAxis.getX() + cylinderPosition.getX();
            double sphereProjectionOnCylinderAxisY = positionOnCylinderAxis * cylinderAxis.getY() + cylinderPosition.getY();
            double sphereProjectionOnCylinderAxisZ = positionOnCylinderAxis * cylinderAxis.getZ() + cylinderPosition.getZ();
            double normalToSphereX = (spherePosition.getX() - sphereProjectionOnCylinderAxisX) / distanceFromCylinderAxis;
            double normalToSphereY = (spherePosition.getY() - sphereProjectionOnCylinderAxisY) / distanceFromCylinderAxis;
            double normalToSphereZ = (spherePosition.getZ() - sphereProjectionOnCylinderAxisZ) / distanceFromCylinderAxis;

            if (firstClosestPointToPack != null)
            {
               firstClosestPointToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);
               firstClosestPointToPack.scaleAdd(sphereRadius, firstClosestPointToPack, spherePosition);
            }

            if (secondClosestPointToPack != null)
            {
               secondClosestPointToPack.set(normalToSphereX, normalToSphereY, normalToSphereZ);
               secondClosestPointToPack.scale(cylinderRadius);
               secondClosestPointToPack.add(sphereProjectionOnCylinderAxisX, sphereProjectionOnCylinderAxisY, sphereProjectionOnCylinderAxisZ);
            }

            if (firstNormalToPack != null)
               firstNormalToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);

            if (secondNormalToPack != null)
               secondNormalToPack.set(normalToSphereX, normalToSphereY, normalToSphereZ);

            return distanceFromCylinderAxis - sphereRadius;
         }
      }
      else
      { // The sphere's center is outside the cylinder's side, maybe touching or not.
         double distanceFromCylinderAxis = Math.sqrt(distanceSquaredFromCylinderAxis);

         double positionOnCylinderAxisClamped = positionOnCylinderAxis;
         if (positionOnCylinderAxisClamped < -cylinderHalfLength)
            positionOnCylinderAxisClamped = -cylinderHalfLength;
         else if (positionOnCylinderAxisClamped > cylinderHalfLength)
            positionOnCylinderAxisClamped = cylinderHalfLength;

         if (positionOnCylinderAxisClamped != positionOnCylinderAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.
            double sphereProjectionOnCylinderAxisX = positionOnCylinderAxis * cylinderAxis.getX() + cylinderPosition.getX();
            double sphereProjectionOnCylinderAxisY = positionOnCylinderAxis * cylinderAxis.getY() + cylinderPosition.getY();
            double sphereProjectionOnCylinderAxisZ = positionOnCylinderAxis * cylinderAxis.getZ() + cylinderPosition.getZ();

            double sphereProjectionOnCylinderAxisClampedX = positionOnCylinderAxisClamped * cylinderAxis.getX() + cylinderPosition.getX();
            double sphereProjectionOnCylinderAxisClampedY = positionOnCylinderAxisClamped * cylinderAxis.getY() + cylinderPosition.getY();
            double sphereProjectionOnCylinderAxisClampedZ = positionOnCylinderAxisClamped * cylinderAxis.getZ() + cylinderPosition.getZ();

            dx = spherePosition.getX() - sphereProjectionOnCylinderAxisClampedX;
            dy = spherePosition.getY() - sphereProjectionOnCylinderAxisClampedY;
            dz = spherePosition.getZ() - sphereProjectionOnCylinderAxisClampedZ;
            double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

            double normalToSphereX = (spherePosition.getX() - sphereProjectionOnCylinderAxisClampedX) / distance;
            double normalToSphereY = (spherePosition.getY() - sphereProjectionOnCylinderAxisClampedY) / distance;
            double normalToSphereZ = (spherePosition.getZ() - sphereProjectionOnCylinderAxisClampedZ) / distance;

            if (firstClosestPointToPack != null)
            {
               firstClosestPointToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);
               firstClosestPointToPack.scaleAdd(sphereRadius, firstClosestPointToPack, spherePosition);
            }

            if (secondClosestPointToPack != null)
            {
               secondClosestPointToPack.set(sphereProjectionOnCylinderAxisX, sphereProjectionOnCylinderAxisY, sphereProjectionOnCylinderAxisZ);
               secondClosestPointToPack.sub(spherePosition);
               secondClosestPointToPack.scale(cylinderRadius / distanceFromCylinderAxis);
               secondClosestPointToPack.add(sphereProjectionOnCylinderAxisClampedX, sphereProjectionOnCylinderAxisClampedY,
                                            sphereProjectionOnCylinderAxisClampedZ);
            }

            if (firstNormalToPack != null)
               firstNormalToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);

            if (secondNormalToPack != null)
               secondNormalToPack.set(normalToSphereX, normalToSphereY, normalToSphereZ);

            return distance - sphereRadius;
         }
         else
         { // Closest point is on the cylinder's side.
            double sphereProjectionOnCylinderAxisX = positionOnCylinderAxis * cylinderAxis.getX() + cylinderPosition.getX();
            double sphereProjectionOnCylinderAxisY = positionOnCylinderAxis * cylinderAxis.getY() + cylinderPosition.getY();
            double sphereProjectionOnCylinderAxisZ = positionOnCylinderAxis * cylinderAxis.getZ() + cylinderPosition.getZ();

            double normalToSphereX = (spherePosition.getX() - sphereProjectionOnCylinderAxisX) / distanceFromCylinderAxis;
            double normalToSphereY = (spherePosition.getY() - sphereProjectionOnCylinderAxisY) / distanceFromCylinderAxis;
            double normalToSphereZ = (spherePosition.getZ() - sphereProjectionOnCylinderAxisZ) / distanceFromCylinderAxis;

            if (firstClosestPointToPack != null)
            {
               firstClosestPointToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);
               firstClosestPointToPack.scaleAdd(sphereRadius, firstClosestPointToPack, spherePosition);
            }

            if (secondClosestPointToPack != null)
            {
               secondClosestPointToPack.set(normalToSphereX, normalToSphereY, normalToSphereZ);
               secondClosestPointToPack.scale(cylinderRadius);
               secondClosestPointToPack.add(sphereProjectionOnCylinderAxisX, sphereProjectionOnCylinderAxisY, sphereProjectionOnCylinderAxisZ);
            }

            if (firstNormalToPack != null)
               firstNormalToPack.set(-normalToSphereX, -normalToSphereY, -normalToSphereZ);

            if (secondNormalToPack != null)
               secondNormalToPack.set(normalToSphereX, normalToSphereY, normalToSphereZ);

            return distanceFromCylinderAxis - sphereRadius;
         }
      }
   }

   public static boolean isPoint3DInsideTorus3D(double torus3DRadius, double torus3DTubeRadius, Point3DReadOnly query, double epsilon)
   {
      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();

      double xyLengthSquared = EuclidCoreTools.normSquared(x, y);

      double outerRadius = torus3DRadius + torus3DTubeRadius + epsilon;
      double innerRadius = torus3DRadius - torus3DTubeRadius - epsilon;

      if (xyLengthSquared > outerRadius * outerRadius || xyLengthSquared < innerRadius * innerRadius)
         return false;

      double xyScale = torus3DRadius / Math.sqrt(xyLengthSquared);

      double closestTubeCenterX = x * xyScale;
      double closestTubeCenterY = y * xyScale;

      return EuclidGeometryTools.distanceBetweenPoint3Ds(x, y, z, closestTubeCenterX, closestTubeCenterY, 0.0) <= torus3DTubeRadius + epsilon;
   }

   public static double signedDistanceBetweenPoint3DAndTorus3D(double torus3DRadius, double torus3DTubeRadius, Point3DReadOnly query)
   {
      return doPoint3DTorus3DCollisionTest(torus3DRadius, torus3DTubeRadius, query, null, null);
   }

   public static boolean orthogonalProjectionOntoTorus3D(double torus3DRadius, double torus3DTubeRadius, Point3DReadOnly pointToProject,
                                                         Point3DBasics projectionToPack)
   {
      return doPoint3DTorus3DCollisionTest(torus3DRadius, torus3DTubeRadius, pointToProject, projectionToPack, null) <= 0.0;
   }

   public static double doPoint3DTorus3DCollisionTest(double torus3DRadius, double torus3DTubeRadius, Point3DReadOnly query, Point3DBasics closestPointToPack,
                                                      Vector3DBasics normalToPack)
   {
      double x = query.getX();
      double y = query.getY();
      double z = query.getZ();
      double xyLengthSquared = normSquared(x, y);

      if (xyLengthSquared < 1.0e-12)
      {
         double xzLength = Math.sqrt(normSquared(torus3DRadius, z));

         if (closestPointToPack != null)
         {
            double scale = torus3DTubeRadius / xzLength;
            double closestTubeCenterX = torus3DRadius;
            double closestTubeCenterY = 0.0;

            double tubeCenterToSurfaceX = -torus3DRadius * scale;
            double tubeCenterToSurfaceY = 0.0;
            double tubeCenterToSurfaceZ = z * scale;

            closestPointToPack.set(closestTubeCenterX, closestTubeCenterY, 0.0);
            closestPointToPack.add(tubeCenterToSurfaceX, tubeCenterToSurfaceY, tubeCenterToSurfaceZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(-torus3DRadius, 0.0, z);
            normalToPack.scale(1.0 / xzLength);
         }

         return xzLength - torus3DTubeRadius;
      }
      else
      {
         double xyScale = torus3DRadius / Math.sqrt(xyLengthSquared);

         double closestTubeCenterX = x * xyScale;
         double closestTubeCenterY = y * xyScale;

         double dx = x - closestTubeCenterX;
         double dy = y - closestTubeCenterY;
         double dz = z;

         double distance = Math.sqrt(normSquared(dx, dy, dz));

         double distanceInv = 1.0 / distance;

         if (closestPointToPack != null)
         {
            closestPointToPack.set(dx, dy, dz);
            closestPointToPack.scale(torus3DTubeRadius * distanceInv);
            closestPointToPack.add(closestTubeCenterX, closestTubeCenterY, 0.0);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(distanceInv);
         }

         return distance - torus3DTubeRadius;
      }
   }

   public static boolean isPoint3DInsideCapsule3D(double capsule3DLength, double capsule3DRadius, Tuple3DReadOnly capsule3DPosition,
                                                  Vector3DReadOnly capsule3DAxis, Point3DReadOnly query, double epsilon)
   {
      double capsule3DHalfLength = 0.5 * capsule3DLength;
      double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
      double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
      double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();

      double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
      double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
      double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();

      double distanceSquared = EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(query.getX(), query.getY(), query.getZ(), topCenterX, topCenterY,
                                                                                             topCenterZ, bottomCenterX, bottomCenterY, bottomCenterZ);
      double upsizedRadius = capsule3DRadius + epsilon;
      return distanceSquared <= upsizedRadius * upsizedRadius;
   }

   public static double signedDistanceBetweenPoint3DAndCapsule3D(double capsule3DLength, double capsule3DRadius, Tuple3DReadOnly capsule3DPosition,
                                                                 Vector3DReadOnly capsule3DAxis, Point3DReadOnly query)
   {
      double capsuleHalfLength = 0.5 * capsule3DLength;
      double topCenterX = capsule3DPosition.getX() + capsuleHalfLength * capsule3DAxis.getX();
      double topCenterY = capsule3DPosition.getY() + capsuleHalfLength * capsule3DAxis.getY();
      double topCenterZ = capsule3DPosition.getZ() + capsuleHalfLength * capsule3DAxis.getZ();

      double bottomCenterX = capsule3DPosition.getX() - capsuleHalfLength * capsule3DAxis.getX();
      double bottomCenterY = capsule3DPosition.getY() - capsuleHalfLength * capsule3DAxis.getY();
      double bottomCenterZ = capsule3DPosition.getZ() - capsuleHalfLength * capsule3DAxis.getZ();

      double distanceFromAxis = EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(query.getX(), query.getY(), query.getZ(), topCenterX, topCenterY,
                                                                                       topCenterZ, bottomCenterX, bottomCenterY, bottomCenterZ);
      return distanceFromAxis - capsule3DRadius;
   }

   public static boolean orthogonalProjectionOntoCapsule3D(double capsule3DLength, double capsule3DRadius, Tuple3DReadOnly capsule3DPosition,
                                                           Vector3DReadOnly capsule3DAxis, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      if (capsule3DRadius <= 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }
      else if (capsule3DLength < 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }
      else if (capsule3DLength == 0.0)
      {
         double distanceSquaredFromCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(capsule3DPosition.getX(), capsule3DPosition.getY(),
                                                                                               capsule3DPosition.getZ(), pointToProject);
         if (distanceSquaredFromCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.sub(pointToProject, capsule3DPosition);
         projectionToPack.scale(capsule3DRadius / Math.sqrt(distanceSquaredFromCenter));
         projectionToPack.add(capsule3DPosition);
         return true;
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject.getX(), pointToProject.getY(), pointToProject.getZ(),
                                                                          capsule3DPosition.getX(), capsule3DPosition.getY(), capsule3DPosition.getZ(),
                                                                          capsule3DAxis.getX(), capsule3DAxis.getY(), capsule3DAxis.getZ());

      if (Math.abs(percentageOnAxis) < capsule3DHalfLength)
      {
         double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
         double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
         double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
         double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ,
                                                                                             pointToProject);

         if (distanceSquaredFromAxis <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         projectionToPack.scale(capsule3DRadius / Math.sqrt(distanceSquaredFromAxis));
         projectionToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         return true;
      }
      else if (percentageOnAxis > 0.0)
      {
         double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceSquaredFromTopCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(topCenterX, topCenterY, topCenterZ, pointToProject);

         if (distanceSquaredFromTopCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(topCenterX, topCenterY, topCenterZ);
         projectionToPack.scale(capsule3DRadius / Math.sqrt(distanceSquaredFromTopCenter));
         projectionToPack.add(topCenterX, topCenterY, topCenterZ);
         return true;
      }
      else // if (percentageOnAxis < 0.0)
      {
         double bottomCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double bottomCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double bottomCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceSquaredFromBottomCenter = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(bottomCenterX, bottomCenterY, bottomCenterZ,
                                                                                                     pointToProject);

         if (distanceSquaredFromBottomCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.set(pointToProject);
         projectionToPack.sub(bottomCenterX, bottomCenterY, bottomCenterZ);
         projectionToPack.scale(capsule3DRadius / Math.sqrt(distanceSquaredFromBottomCenter));
         projectionToPack.add(bottomCenterX, bottomCenterY, bottomCenterZ);
         return true;
      }
   }

   public static double doPoint3DCapsule3DCollisionTest(double capsule3DLength, double capsule3DRadius, Tuple3DReadOnly capsule3DPosition,
                                                        Vector3DReadOnly capsule3DAxis, Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack,
                                                        Vector3DBasics normalToPack)
   {
      if (capsule3DRadius <= 0.0)
      {
         if (closestPointOnSurfaceToPack != null)
            closestPointOnSurfaceToPack.setToNaN();
         if (normalToPack != null)
            normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength < 0.0)
      {
         if (closestPointOnSurfaceToPack != null)
            closestPointOnSurfaceToPack.setToNaN();
         if (normalToPack != null)
            normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength == 0.0)
      {
         return doPoint3DSphere3DCollisionTest(capsule3DRadius, capsule3DPosition, query, closestPointOnSurfaceToPack, normalToPack);
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(query.getX(), query.getY(), query.getZ(), capsule3DPosition.getX(),
                                                                          capsule3DPosition.getY(), capsule3DPosition.getZ(), capsule3DAxis.getX(),
                                                                          capsule3DAxis.getY(), capsule3DAxis.getZ());

      if (Math.abs(percentageOnAxis) < capsule3DHalfLength)
      {
         double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
         double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
         double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
         double distanceFromAxis = EuclidGeometryTools.distanceBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(query);
            closestPointOnSurfaceToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            closestPointOnSurfaceToPack.scale(capsule3DRadius / distanceFromAxis);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(query);
            normalToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            normalToPack.scale(1.0 / distanceFromAxis);
         }

         return distanceFromAxis - capsule3DRadius;
      }
      else if (percentageOnAxis > 0.0)
      {
         double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceFromTopCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(topCenterX, topCenterY, topCenterZ, query);

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(query);
            closestPointOnSurfaceToPack.sub(topCenterX, topCenterY, topCenterZ);
            closestPointOnSurfaceToPack.scale(capsule3DRadius / distanceFromTopCenter);
            closestPointOnSurfaceToPack.add(topCenterX, topCenterY, topCenterZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(query);
            normalToPack.sub(topCenterX, topCenterY, topCenterZ);
            normalToPack.scale(1.0 / distanceFromTopCenter);
         }

         return distanceFromTopCenter - capsule3DRadius;
      }
      else // if (percentageOnAxis < 0.0)
      {
         double bottomCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double bottomCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double bottomCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceFromBottomCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(bottomCenterX, bottomCenterY, bottomCenterZ, query);

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(query);
            closestPointOnSurfaceToPack.sub(bottomCenterX, bottomCenterY, bottomCenterZ);
            closestPointOnSurfaceToPack.scale(capsule3DRadius / distanceFromBottomCenter);
            closestPointOnSurfaceToPack.add(bottomCenterX, bottomCenterY, bottomCenterZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(query);
            normalToPack.sub(bottomCenterX, bottomCenterY, bottomCenterZ);
            normalToPack.scale(1.0 / distanceFromBottomCenter);
         }

         return distanceFromBottomCenter - capsule3DRadius;
      }
   }
}
