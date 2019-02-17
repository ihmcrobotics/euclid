package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeTools
{
   private static final double SPHERE_SMALLEST_DISTANCE_TO_ORIGIN = 1.0e-12;
   private static final double TORUS_SMALLEST_DISTANCE_TO_AXIS = 1.0e-12;

   public static boolean isPoint3DInsideBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize, double epsilon)
   {
      return isPoint3DInsideBox3D(query.getX(), query.getY(), query.getZ(), box3DSize, epsilon);
   }

   public static boolean isPoint3DInsideBox3D(double x, double y, double z, Vector3DReadOnly box3DSize, double epsilon)
   {
      if (Math.abs(x) <= 0.5 * box3DSize.getX() + epsilon)
      {
         if (Math.abs(y) <= 0.5 * box3DSize.getY() + epsilon)
         {
            return Math.abs(z) <= 0.5 * box3DSize.getZ() + epsilon;
         }
      }
      return false;
   }

   public static double signedDistanceBetweenPoint3DAndBox3D(Point3DReadOnly query, Vector3DReadOnly box3DSize)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      boolean isInside = Math.abs(query.getX()) <= halfSizeX && Math.abs(query.getY()) <= halfSizeY && Math.abs(query.getZ()) <= halfSizeZ;

      if (isInside)
      {
         double dx = Math.abs(Math.abs(query.getX()) - halfSizeX);
         double dy = Math.abs(Math.abs(query.getY()) - halfSizeY);
         double dz = Math.abs(Math.abs(query.getZ()) - halfSizeZ);

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         double dx = query.getX() - EuclidCoreTools.clamp(query.getX(), halfSizeX);
         double dy = query.getY() - EuclidCoreTools.clamp(query.getY(), halfSizeY);
         double dz = query.getZ() - EuclidCoreTools.clamp(query.getZ(), halfSizeZ);

         return Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));
      }
   }

   public static boolean orthogonalProjectionOntoBox3D(Point3DReadOnly pointToProject, Vector3DReadOnly box3DSize, Point3DBasics projectionToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double xLocal = pointToProject.getX();
      double yLocal = pointToProject.getY();
      double zLocal = pointToProject.getZ();

      if (Math.abs(xLocal) > halfSizeX || Math.abs(yLocal) > halfSizeY || Math.abs(zLocal) > halfSizeZ)
      {
         double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
         double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
         double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

         projectionToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);

         return true;
      }
      else
      {
         return false;
      }
   }

   public static void supportingVertexBox3D(Vector3DReadOnly supportDirection, Vector3DReadOnly box3DSize, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.setX(supportDirection.getX() > 0.0 ? box3DSize.getX() : -box3DSize.getX());
      supportingVertexToPack.setY(supportDirection.getY() > 0.0 ? box3DSize.getY() : -box3DSize.getY());
      supportingVertexToPack.setZ(supportDirection.getZ() > 0.0 ? box3DSize.getZ() : -box3DSize.getZ());
      supportingVertexToPack.scale(0.5);
   }

   public static double doPoint3DBox3DCollisionTest(Point3DReadOnly query, Vector3DReadOnly box3DSize, Point3DBasics closestPointToPack,
                                                    Vector3DBasics normalToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double dx, dy, dz;
      double xLocal = query.getX();
      double yLocal = query.getY();
      double zLocal = query.getZ();

      boolean isInside = Math.abs(xLocal) <= halfSizeX && Math.abs(yLocal) <= halfSizeY && Math.abs(zLocal) <= halfSizeZ;

      if (isInside)
      {
         dx = Math.abs(Math.abs(xLocal) - halfSizeX);
         dy = Math.abs(Math.abs(yLocal) - halfSizeY);
         dz = Math.abs(Math.abs(zLocal) - halfSizeZ);

         closestPointToPack.set(xLocal, yLocal, zLocal);

         if (dx < dy)
         {
            if (dx < dz)
               closestPointToPack.setX(Math.copySign(halfSizeX, xLocal));
            else
               closestPointToPack.setZ(Math.copySign(halfSizeZ, zLocal));
         }
         else
         {
            if (dy < dz)
               closestPointToPack.setY(Math.copySign(halfSizeY, yLocal));
            else
               closestPointToPack.setZ(Math.copySign(halfSizeZ, zLocal));
         }
         normalToPack.setToZero();

         if (dx < dy)
         {
            if (dx < dz)
               normalToPack.setX(Math.copySign(1.0, xLocal));
            else
               normalToPack.setZ(Math.copySign(1.0, zLocal));
         }
         else
         {
            if (dy < dz)
               normalToPack.setY(Math.copySign(1.0, yLocal));
            else
               normalToPack.setZ(Math.copySign(1.0, zLocal));
         }

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
         double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
         double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

         dx = xLocal - xLocalClamped;
         dy = yLocal - yLocalClamped;
         dz = zLocal - zLocalClamped;

         double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

         closestPointToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);

         return distance;
      }
   }

   public static boolean isPoint3DInsideCapsule3D(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                  double capsule3DLength, double capsule3DRadius, double epsilon)
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

   public static double signedDistanceBetweenPoint3DAndCapsule3D(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                                 double capsule3DLength, double capsule3DRadius)
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

   public static boolean orthogonalProjectionOntoCapsule3D(Point3DReadOnly pointToProject, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                           double capsule3DLength, double capsule3DRadius, Point3DBasics projectionToPack)
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
         double distanceSquaredFromCenter = capsule3DPosition.distanceSquared(pointToProject);
         if (distanceSquaredFromCenter <= capsule3DRadius * capsule3DRadius)
            return false;

         projectionToPack.sub(pointToProject, capsule3DPosition);
         projectionToPack.scale(capsule3DRadius / Math.sqrt(distanceSquaredFromCenter));
         projectionToPack.add(capsule3DPosition);
         return true;
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject, capsule3DPosition, capsule3DAxis);

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
         double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
         double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
         double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();
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

   public static void supportingVertexCapsule3D(Vector3DReadOnly supportDirection, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                double capsule3DLength, double capsule3DRadius, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.setAndScale(capsule3DRadius / supportDirection.length(), supportDirection);
      supportingVertexToPack.add(capsule3DPosition);

      if (supportDirection.dot(capsule3DAxis) > 0.0)
         supportingVertexToPack.scaleAdd(0.5 * capsule3DLength, capsule3DAxis, supportingVertexToPack);
      else
         supportingVertexToPack.scaleAdd(-0.5 * capsule3DLength, capsule3DAxis, supportingVertexToPack);
   }

   public static void boundingBoxCapsule3D(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength, double capsule3DRadius,
                                           BoundingBox3DBasics boundingBoxToPack)
   {
      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double maxX = Math.abs(capsule3DHalfLength * capsule3DAxis.getX()) + capsule3DRadius;
      double maxY = Math.abs(capsule3DHalfLength * capsule3DAxis.getY()) + capsule3DRadius;
      double maxZ = Math.abs(capsule3DHalfLength * capsule3DAxis.getZ()) + capsule3DRadius;

      double minX = -maxX + capsule3DPosition.getX();
      double minY = -maxY + capsule3DPosition.getY();
      double minZ = -maxZ + capsule3DPosition.getZ();
      maxX += capsule3DPosition.getX();
      maxY += capsule3DPosition.getY();
      maxZ += capsule3DPosition.getZ();

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static double doPoint3DCapsule3DCollisionTest(Point3DReadOnly query, Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis,
                                                        double capsule3DLength, double capsule3DRadius, Point3DBasics closestPointOnSurfaceToPack,
                                                        Vector3DBasics normalToPack)
   {
      if (capsule3DRadius <= 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength < 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }
      else if (capsule3DLength == 0.0)
      {
         return doPoint3DSphere3DCollisionTest(query, capsule3DPosition, capsule3DRadius, closestPointOnSurfaceToPack, normalToPack);
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, capsule3DPosition, capsule3DAxis);

      if (Math.abs(percentageOnAxis) < capsule3DHalfLength)
      {
         double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
         double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
         double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
         double distanceFromAxis = EuclidGeometryTools.distanceBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

         normalToPack.set(query);
         normalToPack.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
         normalToPack.scale(1.0 / distanceFromAxis);

         closestPointOnSurfaceToPack.setAndScale(capsule3DRadius, normalToPack);
         closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

         return distanceFromAxis - capsule3DRadius;
      }
      else if (percentageOnAxis > 0.0)
      {
         double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
         double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
         double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceFromTopCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(topCenterX, topCenterY, topCenterZ, query);

         normalToPack.set(query);
         normalToPack.sub(topCenterX, topCenterY, topCenterZ);
         normalToPack.scale(1.0 / distanceFromTopCenter);

         closestPointOnSurfaceToPack.setAndScale(capsule3DRadius, normalToPack);
         closestPointOnSurfaceToPack.add(topCenterX, topCenterY, topCenterZ);

         return distanceFromTopCenter - capsule3DRadius;
      }
      else // if (percentageOnAxis < 0.0)
      {
         double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
         double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
         double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();
         double distanceFromBottomCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(bottomCenterX, bottomCenterY, bottomCenterZ, query);

         normalToPack.set(query);
         normalToPack.sub(bottomCenterX, bottomCenterY, bottomCenterZ);
         normalToPack.scale(1.0 / distanceFromBottomCenter);

         closestPointOnSurfaceToPack.setAndScale(capsule3DRadius, normalToPack);
         closestPointOnSurfaceToPack.add(bottomCenterX, bottomCenterY, bottomCenterZ);

         return distanceFromBottomCenter - capsule3DRadius;
      }
   }

   public static boolean isPoint3DInsideCylinder3D(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                   double cylinder3DLength, double cylinder3DRadius, double epsilon)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double halfLengthPlusEpsilon = 0.5 * cylinder3DLength + epsilon;

      if (Math.abs(positionOnAxis) > halfLengthPlusEpsilon)
         return false;

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);
      double radiusWithEpsilon = cylinder3DRadius + epsilon;
      return distanceSquaredFromAxis <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndCylinder3D(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                                  double cylinder3DLength, double cylinder3DRadius)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         return Double.NaN;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double axisToQueryX = query.getX() - (cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX());
      double axisToQueryY = query.getY() - (cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY());
      double axisToQueryZ = query.getZ() - (cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ());
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            return -(positionOnAxis + halfLength);
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            return positionOnAxis - halfLength;
         }

         // The query is inside the cylinder
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);
         double dh = halfLength - Math.abs(positionOnAxis);
         double dr = cylinder3DRadius - distanceFromAxis;

         if (dh < dr)
         {
            if (positionOnAxis < 0)
            { // Closer to the bottom face
               return -(positionOnAxis + halfLength);
            }
            else
            { // Closer to the top face
               return positionOnAxis - halfLength;
            }
         }
         else
         { // Closer to the cylinder part
            return distanceFromAxis - cylinder3DRadius;
         }
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            return EuclidGeometryTools.distanceBetweenPoint3Ds(closestX, closestY, closestZ, query);
         }
         else
         { // Closest point is on the cylinder.
            return distanceFromAxis - cylinder3DRadius;
         }
      }
   }

   public static boolean orthogonalProjectionOntoCylinder3D(Point3DReadOnly pointToProject, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                            double cylinder3DLength, double cylinder3DRadius, Point3DBasics projectionToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         projectionToPack.setToNaN();
         return false;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(pointToProject, cylinder3DPosition, cylinder3DAxis);

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();

      double axisToQueryX = pointToProject.getX() - projectionOnAxisX;
      double axisToQueryY = pointToProject.getY() - projectionOnAxisY;
      double axisToQueryZ = pointToProject.getZ() - projectionOnAxisZ;
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            projectionToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, pointToProject);
            return true;
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            projectionToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, pointToProject);
            return true;
         }

         // The query is inside the cylinder
         return false;
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.
            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            projectionToPack.set(closestX, closestY, closestZ);
            return true;
         }
         else
         { // Closest point is on the cylinder.
            projectionToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            projectionToPack.scale(cylinder3DRadius / distanceFromAxis);
            projectionToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            return true;
         }
      }
   }

   public static void supportingVertexCylinder3D(Vector3DReadOnly supportDirection, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                 double cylinder3DLength, double cylinder3DRadius, Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(supportDirection);
      double dot = supportDirection.dot(cylinder3DAxis);
      supportingVertexToPack.setAndScale(dot, cylinder3DAxis);
      supportingVertexToPack.sub(supportDirection, supportingVertexToPack);
      supportingVertexToPack.scale(cylinder3DRadius / supportingVertexToPack.distanceFromOrigin());
      supportingVertexToPack.add(cylinder3DPosition);

      if (supportDirection.dot(cylinder3DAxis) > 0.0)
         supportingVertexToPack.scaleAdd(0.5 * cylinder3DLength, cylinder3DAxis, supportingVertexToPack);
      else
         supportingVertexToPack.scaleAdd(-0.5 * cylinder3DLength, cylinder3DAxis, supportingVertexToPack);
   }

   public static void boundingBoxCylinder3D(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                            double cylinder3DRadius, BoundingBox3DBasics boundingBoxToPack)
   {
      // From https://iquilezles.org/www/articles/diskbbox/diskbbox.htm
      double cylinder3DHalfLength = 0.5 * cylinder3DLength;

      double normSquared = cylinder3DAxis.lengthSquared();
      double capMinMaxX = Math.max(0.0, cylinder3DRadius * Math.sqrt(1.0 - cylinder3DAxis.getX() * cylinder3DAxis.getX() / normSquared));
      double capMinMaxY = Math.max(0.0, cylinder3DRadius * Math.sqrt(1.0 - cylinder3DAxis.getY() * cylinder3DAxis.getY() / normSquared));
      double capMinMaxZ = Math.max(0.0, cylinder3DRadius * Math.sqrt(1.0 - cylinder3DAxis.getZ() * cylinder3DAxis.getZ() / normSquared));

      double maxX = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getX()) + capMinMaxX;
      double maxY = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getY()) + capMinMaxY;
      double maxZ = Math.abs(cylinder3DHalfLength * cylinder3DAxis.getZ()) + capMinMaxZ;

      double minX = -maxX + cylinder3DPosition.getX();
      double minY = -maxY + cylinder3DPosition.getY();
      double minZ = -maxZ + cylinder3DPosition.getZ();
      maxX += cylinder3DPosition.getX();
      maxY += cylinder3DPosition.getY();
      maxZ += cylinder3DPosition.getZ();

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);

      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static double doPoint3DCylinder3DCollisionTest(Point3DReadOnly query, Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis,
                                                         double cylinder3DLength, double cylinder3DRadius, Point3DBasics closestPointOnSurfaceToPack,
                                                         Vector3DBasics normalToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         closestPointOnSurfaceToPack.setToNaN();
         normalToPack.setToNaN();
         return Double.NaN;
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, cylinder3DPosition, cylinder3DAxis);

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();

      double axisToQueryX = query.getX() - projectionOnAxisX;
      double axisToQueryY = query.getY() - projectionOnAxisY;
      double axisToQueryZ = query.getZ() - projectionOnAxisZ;
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
            normalToPack.setAndNegate(cylinder3DAxis);
            return -(positionOnAxis + halfLength);
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
            normalToPack.set(cylinder3DAxis);
            return positionOnAxis - halfLength;
         }

         // The query is inside the cylinder
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);
         double dh = halfLength - Math.abs(positionOnAxis);
         double dr = cylinder3DRadius - distanceFromAxis;

         if (dh < dr)
         {
            if (positionOnAxis < 0)
            { // Closer to the bottom face
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
               normalToPack.setAndNegate(cylinder3DAxis);
               return -(positionOnAxis + halfLength);
            }
            else
            { // Closer to the top face
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
               normalToPack.set(cylinder3DAxis);
               return positionOnAxis - halfLength;
            }
         }
         else
         { // Closer to the cylinder part
            normalToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            normalToPack.scale(1.0 / distanceFromAxis);

            closestPointOnSurfaceToPack.setAndScale(cylinder3DRadius, normalToPack);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

            return distanceFromAxis - cylinder3DRadius;
         }
      }
      else
      { // The query is outside and closest to the cylinder's side.
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         double positionOnAxisClamped = positionOnAxis;
         if (positionOnAxisClamped < -halfLength)
            positionOnAxisClamped = -halfLength;
         else if (positionOnAxisClamped > halfLength)
            positionOnAxisClamped = halfLength;

         if (positionOnAxisClamped != positionOnAxis)
         { // Closest point is on the circle adjacent to the cylinder and top or bottom face.

            double projectionOnAxisXClamped = cylinder3DPosition.getX() + positionOnAxisClamped * cylinder3DAxis.getX();
            double projectionOnAxisYClamped = cylinder3DPosition.getY() + positionOnAxisClamped * cylinder3DAxis.getY();
            double projectionOnAxisZClamped = cylinder3DPosition.getZ() + positionOnAxisClamped * cylinder3DAxis.getZ();

            double toCylinderScale = cylinder3DRadius / distanceFromAxis;
            double closestX = axisToQueryX * toCylinderScale + projectionOnAxisXClamped;
            double closestY = axisToQueryY * toCylinderScale + projectionOnAxisYClamped;
            double closestZ = axisToQueryZ * toCylinderScale + projectionOnAxisZClamped;
            double dX = query.getX() - closestX;
            double dY = query.getY() - closestY;
            double dZ = query.getZ() - closestZ;
            double distance = Math.sqrt(EuclidCoreTools.normSquared(dX, dY, dZ));

            closestPointOnSurfaceToPack.set(closestX, closestY, closestZ);

            if (distance < 1.0e-12)
            {
               if (positionOnAxis > 0.0)
                  normalToPack.set(cylinder3DAxis);
               else
                  normalToPack.setAndNegate(cylinder3DAxis);
            }
            else
            {
               normalToPack.set(dX, dY, dZ);
               normalToPack.scale(1.0 / distance);
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            normalToPack.set(axisToQueryX, axisToQueryY, axisToQueryZ);
            normalToPack.scale(1.0 / distanceFromAxis);

            closestPointOnSurfaceToPack.setAndScale(cylinder3DRadius, normalToPack);
            closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

            return distanceFromAxis - cylinder3DRadius;
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
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double scaleFactor = 1.0 / Math.sqrt(EuclidCoreTools.normSquared(query.getX() / xRadius, query.getY() / yRadius, query.getZ() / zRadius));

      return query.distanceFromOrigin() * (1.0 - scaleFactor);
   }

   public static boolean orthogonalProjectionOntoEllipsoid3D(Point3DReadOnly pointToProject, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics projectionToPack)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double sumOfSquares = EuclidCoreTools.normSquared(pointToProject.getX() / xRadius, pointToProject.getY() / yRadius, pointToProject.getZ() / zRadius);

      if (sumOfSquares > 1.0)
      {
         projectionToPack.scale(1.0 / Math.sqrt(sumOfSquares));
         return true;
      }
      else
      {
         return false;
      }
   }

   public static void supportingVertexEllipsoid3D(Vector3DReadOnly supportDirection, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics supportingVertexToPack)
   {
      double nx = supportDirection.getX();
      double ny = supportDirection.getY();
      double nz = supportDirection.getZ();
      double nLength = Math.sqrt(EuclidCoreTools.normSquared(nx, ny, nz));
      nx *= ellipsoid3DRadii.getX() / nLength;
      ny *= ellipsoid3DRadii.getY() / nLength;
      nz *= ellipsoid3DRadii.getZ() / nLength;

      supportingVertexToPack.set(nx, ny, nz);
      supportingVertexToPack.scale(ellipsoid3DRadii.getX(), ellipsoid3DRadii.getY(), ellipsoid3DRadii.getZ());
      supportingVertexToPack.scale(1.0 / Math.sqrt(EuclidCoreTools.normSquared(nx, ny, nz)));
   }

   public static void boundingBoxEllipsoid3D(Point3DReadOnly ellipsoid3DPosition, RotationMatrixReadOnly ellipsoid3DOrientation,
                                             Vector3DReadOnly ellipsoid3DRadii, BoundingBox3DBasics boundingBoxToPack)
   {
      double m00 = ellipsoid3DOrientation.getM00() * ellipsoid3DOrientation.getM00();
      double m01 = ellipsoid3DOrientation.getM01() * ellipsoid3DOrientation.getM01();
      double m02 = ellipsoid3DOrientation.getM02() * ellipsoid3DOrientation.getM02();
      double m10 = ellipsoid3DOrientation.getM10() * ellipsoid3DOrientation.getM10();
      double m11 = ellipsoid3DOrientation.getM11() * ellipsoid3DOrientation.getM11();
      double m12 = ellipsoid3DOrientation.getM12() * ellipsoid3DOrientation.getM12();
      double m20 = ellipsoid3DOrientation.getM20() * ellipsoid3DOrientation.getM20();
      double m21 = ellipsoid3DOrientation.getM21() * ellipsoid3DOrientation.getM21();
      double m22 = ellipsoid3DOrientation.getM22() * ellipsoid3DOrientation.getM22();

      double rx = ellipsoid3DRadii.getX() * ellipsoid3DRadii.getX();
      double ry = ellipsoid3DRadii.getY() * ellipsoid3DRadii.getY();
      double rz = ellipsoid3DRadii.getZ() * ellipsoid3DRadii.getZ();

      double xRange = Math.sqrt(m00 * rx + m01 * ry + m02 * rz);
      double yRange = Math.sqrt(m10 * rx + m11 * ry + m12 * rz);
      double zRange = Math.sqrt(m20 * rx + m21 * ry + m22 * rz);

      double maxX = ellipsoid3DPosition.getX() + xRange;
      double maxY = ellipsoid3DPosition.getY() + yRange;
      double maxZ = ellipsoid3DPosition.getZ() + zRange;
      double minX = ellipsoid3DPosition.getX() - xRange;
      double minY = ellipsoid3DPosition.getY() - yRange;
      double minZ = ellipsoid3DPosition.getZ() - zRange;
      boundingBoxToPack.set(minX, minY, minZ, maxX, maxY, maxZ);
   }

   public static double doPoint3DEllipsoid3DCollisionTest(Point3DReadOnly query, Vector3DReadOnly ellipsoid3DRadii, Point3DBasics closestPointToPack,
                                                          Vector3DBasics normalToPack)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double sumOfSquares = EuclidCoreTools.normSquared(query.getX() / xRadius, query.getY() / yRadius, query.getZ() / zRadius);

      if (sumOfSquares > 1.0e-10)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

         closestPointToPack.setAndScale(scaleFactor, query);

         double xScale = 1.0 / (xRadius * xRadius);
         double yScale = 1.0 / (yRadius * yRadius);
         double zScale = 1.0 / (zRadius * zRadius);

         normalToPack.set(query);
         normalToPack.scale(xScale, yScale, zScale);
         normalToPack.normalize();

         return query.distanceFromOrigin() * (1.0 - scaleFactor);
      }
      else
      {
         closestPointToPack.set(0.0, 0.0, ellipsoid3DRadii.getZ());
         normalToPack.set(Axis.Z);

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

   public static double computeRamp3DIncline(Vector3DReadOnly ramp3DSize)
   {
      return computeRanp3DIncline(ramp3DSize.getX(), ramp3DSize.getZ());
   }

   public static double computeRanp3DIncline(double ramp3DSizeX, double ramp3DSizeZ)
   {
      return Math.atan(ramp3DSizeZ / ramp3DSizeX);
   }

   public static boolean isPoint3DInsideRamp3D(Point3DReadOnly query, Vector3DReadOnly ramp3DSize, double epsilon)
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
      return rampDirectionX * query.getZ() - query.getX() * rampDirectionZ <= epsilon;
   }

   public static double signedDistanceBetweenPoint3DAndRamp3D(Point3DReadOnly query, Vector3DReadOnly ramp3DSize)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (query.getZ() < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(query.getX(), 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = 0.0;

         if (xClosest == query.getX() && yClosest == query.getY())
            return -query.getZ();
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (query.getX() > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(query.getZ(), 0.0, ramp3DSize.getZ());

         if (yClosest == query.getY() && zClosest == query.getZ())
            return query.getX() - ramp3DSize.getX();
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(query.getY(), -halfWidth, halfWidth);
         double zClosest = 0.0;

         return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
      }
      else if (Math.abs(query.getY()) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, query.getY());

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            return Math.abs(query.getY()) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = query.getX() * rampDirectionX + query.getZ() * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            return EuclidGeometryTools.distanceBetweenPoint3Ds(xClosest, yClosest, zClosest, query);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(query.getX(), query.getZ(), 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         return rampDirectionX * query.getZ() - query.getX() * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - query.getY());
         double distanceToLeftFace = halfWidth - query.getY();
         double distanceToRearFace = ramp3DSize.getX() - query.getX();
         double distanceToBottomFace = query.getZ();
         double distanceToSlopeFace = -(rampDirectionX * query.getZ() - query.getX() * rampDirectionZ);

         double minDistance = distanceToRightFace;
         if (minDistance > distanceToLeftFace)
            minDistance = distanceToLeftFace;
         if (minDistance > distanceToRearFace)
            minDistance = distanceToRearFace;
         if (minDistance > distanceToBottomFace)
            minDistance = distanceToBottomFace;
         if (minDistance > distanceToSlopeFace)
            minDistance = distanceToSlopeFace;
         return -minDistance;
      }
   }

   public static boolean orthogonalProjectionOntoRamp3D(Point3DReadOnly pointToProject, Vector3DReadOnly ramp3DSize, Point3DBasics projectionToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double xQuery = pointToProject.getX();
      double yQuery = pointToProject.getY();
      double zQuery = pointToProject.getZ();

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (xQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zQuery, 0.0, ramp3DSize.getZ());

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else if (Math.abs(yQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xQuery;
            double zClosest = zQuery;

            projectionToPack.set(xClosest, yClosest, zClosest);
            return true;
         }
         else
         { // Query is above the slope
            double dot = xQuery * rampDirectionX + zQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            projectionToPack.set(xClosest, yClosest, zClosest);
            return true;
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xQuery, zQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xQuery * rampDirectionX + zQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yQuery;
         double zClosest = dot * rampDirectionZ;

         projectionToPack.set(xClosest, yClosest, zClosest);
         return true;
      }
      else
      { // Query is inside the ramp
         return false;
      }
   }

   public static void supportingVectexRamp3D(Vector3DReadOnly supportDirection, Vector3DReadOnly ramp3DSize, Point3DBasics supportingVertexToPack)
   {
      if (supportDirection.getZ() < 0.0)
      {
         supportingVertexToPack.setX(supportDirection.getX() > 0.0 ? ramp3DSize.getX() : 0.0);
         supportingVertexToPack.setZ(0.0);
      }
      else if (supportDirection.getX() > 0.0)
      {
         supportingVertexToPack.setX(ramp3DSize.getX());
         supportingVertexToPack.setZ(supportDirection.getZ() > 0.0 ? ramp3DSize.getZ() : 0.0);
      }
      else
      { // Look at a the incline of the direction.
         double directionInclineTanArgument = -supportDirection.getX() / supportDirection.getZ();
         double rampInclineTanArgument = ramp3DSize.getZ() / ramp3DSize.getX();

         if (directionInclineTanArgument > rampInclineTanArgument)
         { // Pointing toward the bottom of the ramp
            supportingVertexToPack.setX(0.0);
            supportingVertexToPack.setZ(0.0);
         }
         else
         { // Pointing toward the top of the ramp
            supportingVertexToPack.setX(ramp3DSize.getX());
            supportingVertexToPack.setZ(ramp3DSize.getZ());
         }
      }

      supportingVertexToPack.setY(supportDirection.getY() > 0.0 ? 0.5 * ramp3DSize.getY() : -0.5 * ramp3DSize.getY());
   }

   public static double doPoint3DRamp3DCollisionTest(Point3DReadOnly query, Vector3DReadOnly ramp3DSize, Point3DBasics closestPointOnSurfaceToPack,
                                                     Vector3DBasics normalToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double xLocalQuery = query.getX();
      double yLocalQuery = query.getY();
      double zLocalQuery = query.getZ();

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

         if (xClosest == xLocalQuery && yClosest == yLocalQuery)
         {
            normalToPack.setAndNegate(Axis.Z);
            return -zLocalQuery;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

         if (yClosest == yLocalQuery && zClosest == zLocalQuery)
         {
            normalToPack.set(Axis.X);
            return xLocalQuery - ramp3DSize.getX();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xLocalQuery;
            double zClosest = zLocalQuery;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);

            if (yLocalQuery >= 0.0)
               normalToPack.set(Axis.Y);
            else
               normalToPack.setAndNegate(Axis.Y);

            return Math.abs(yLocalQuery) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            return computeNormalAndDistanceFromClosestPoint(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yLocalQuery;
         double zClosest = dot * rampDirectionZ;

         closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
         normalToPack.set(rampNormalX, 0.0, rampNormalZ);

         return rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ;
      }
      else
      { // Query is inside the ramp
         double distanceToRightFace = -(-halfWidth - yLocalQuery);
         double distanceToLeftFace = halfWidth - yLocalQuery;
         double distanceToRearFace = ramp3DSize.getX() - xLocalQuery;
         double distanceToBottomFace = zLocalQuery;
         double distanceToSlopeFace = -(rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            closestPointOnSurfaceToPack.set(xLocalQuery, -halfWidth, zLocalQuery);
            normalToPack.setAndNegate(Axis.Y);
            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            closestPointOnSurfaceToPack.set(xLocalQuery, halfWidth, zLocalQuery);
            normalToPack.set(Axis.Y);
            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            closestPointOnSurfaceToPack.set(ramp3DSize.getX(), yLocalQuery, zLocalQuery);
            normalToPack.set(Axis.X);
            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            closestPointOnSurfaceToPack.set(xLocalQuery, yLocalQuery, 0.0);
            normalToPack.setAndNegate(Axis.Z);
            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double yClosest = yLocalQuery;
            double zClosest = dot * rampDirectionZ;

            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            return -distanceToSlopeFace;
         }
      }
   }

   private static double computeNormalAndDistanceFromClosestPoint(double xLocalQuery, double yLocalQuery, double zLocalQuery, double xLocalClosest,
                                                                  double yLocalClosest, double zLocalClosest, Vector3DBasics normalToPack)
   {
      double dx = xLocalQuery - xLocalClosest;
      double dy = yLocalQuery - yLocalClosest;
      double dz = zLocalQuery - zLocalClosest;

      double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

      normalToPack.set(dx, dy, dz);
      normalToPack.scale(1.0 / distance);

      return distance;
   }

   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3, double value4)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3 && possibleMin <= value4;
   }

   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2, double value3)
   {
      return possibleMin <= value1 && possibleMin <= value2 && possibleMin <= value3;
   }

   public static boolean isFirstValueMinimum(double possibleMin, double value1, double value2)
   {
      return possibleMin <= value1 && possibleMin <= value2;
   }

   public static boolean isPoint3DInsideSphere3D(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius, double epsilon)
   {
      double radiusWithEpsilon = sphere3DRadius + epsilon;
      return sphere3DPosition.distanceSquared(query) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndSphere3D(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius)
   {
      return sphere3DPosition.distance(query) - sphere3DRadius;
   }

   public static boolean orthogonalProjectionOntoSphere3D(Point3DReadOnly pointToProject, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                          Point3DBasics projectionToPack)
   {
      double distanceSquared = sphere3DPosition.distanceSquared(pointToProject);
      if (distanceSquared <= sphere3DRadius * sphere3DRadius)
         return false;

      projectionToPack.sub(pointToProject, sphere3DPosition);
      projectionToPack.scale(sphere3DRadius / Math.sqrt(distanceSquared));
      projectionToPack.add(sphere3DPosition);
      return true;
   }

   public static void supportingVertexSphere3D(Vector3DReadOnly supportDirection, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                               Point3DBasics supportingVertexToPack)
   {
      supportingVertexToPack.set(supportDirection);
      supportingVertexToPack.scale(sphere3DRadius / supportDirection.length());
      supportingVertexToPack.add(sphere3DPosition);
   }

   public static double doPoint3DSphere3DCollisionTest(Point3DReadOnly query, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                       Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double distance = sphere3DPosition.distance(query);

      if (distance > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
      {
         closestPointToPack.sub(query, sphere3DPosition);
         closestPointToPack.scale(sphere3DRadius / distance);
         normalToPack.sub(query, sphere3DPosition);
         normalToPack.scale(1.0 / distance);
      }
      else
      {
         closestPointToPack.set(0.0, 0.0, sphere3DRadius);
         normalToPack.set(0.0, 0.0, 1.0);
      }
      closestPointToPack.add(sphere3DPosition);

      return distance - sphere3DRadius;
   }

   public static boolean isPoint3DInsideTorus3D(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis, double torus3DRadius,
                                                double torus3DTubeRadius, double epsilon)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, torus3DPosition, torus3DAxis);
      double projectionOnAxisX = torus3DPosition.getX() + positionOnAxis * torus3DAxis.getX();
      double projectionOnAxisY = torus3DPosition.getY() + positionOnAxis * torus3DAxis.getY();
      double projectionOnAxisZ = torus3DPosition.getZ() + positionOnAxis * torus3DAxis.getZ();
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

      double tubeRadiusWithEpsilon = torus3DTubeRadius + epsilon;
      double outerRadius = torus3DRadius + tubeRadiusWithEpsilon;
      double innerRadius = torus3DRadius - tubeRadiusWithEpsilon;

      if (distanceSquaredFromAxis > outerRadius * outerRadius || distanceSquaredFromAxis < innerRadius * innerRadius)
         return false;

      double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);
      return EuclidCoreTools.normSquared(distanceFromAxis - torus3DRadius, positionOnAxis) <= tubeRadiusWithEpsilon * tubeRadiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndTorus3D(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                               double torus3DRadius, double torus3DTubeRadius)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, torus3DPosition, torus3DAxis);
      double projectionOnAxisX = torus3DPosition.getX() + positionOnAxis * torus3DAxis.getX();
      double projectionOnAxisY = torus3DPosition.getY() + positionOnAxis * torus3DAxis.getY();
      double projectionOnAxisZ = torus3DPosition.getZ() + positionOnAxis * torus3DAxis.getZ();
      double distanceSquaredFromAxis = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, query);

      if (distanceSquaredFromAxis < 1.0e-12)
      {
         return Math.sqrt(normSquared(torus3DRadius, positionOnAxis)) - torus3DTubeRadius;
      }
      else
      {
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);
         return Math.sqrt(normSquared(distanceFromAxis - torus3DRadius, positionOnAxis)) - torus3DTubeRadius;
      }
   }

   public static boolean orthogonalProjectionOntoTorus3D(Point3DReadOnly pointToProject, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                         double torus3DRadius, double torus3DTubeRadius, Point3DBasics projectionToPack)
   {
      double x = pointToProject.getX() - torus3DPosition.getX();
      double y = pointToProject.getY() - torus3DPosition.getY();
      double z = pointToProject.getZ() - torus3DPosition.getZ();
      double percentageOnAxis = TupleTools.dot(x, y, z, torus3DAxis);

      double xInPlane = x - percentageOnAxis * torus3DAxis.getX();
      double yInPlane = y - percentageOnAxis * torus3DAxis.getY();
      double zInPlane = z - percentageOnAxis * torus3DAxis.getZ();

      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(xInPlane, yInPlane, zInPlane);

      if (distanceSquaredFromAxis < TORUS_SMALLEST_DISTANCE_TO_AXIS)
      { // We need to setup a vector that is orthogonal to the torus' axis, then we'll perform the projection along that vector.
         double xNonCollinearToAxis, yNonCollinearToAxis, zNonCollinearToAxis;

         // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(torus3DAxis.getY()) > 0.1 || Math.abs(torus3DAxis.getZ()) > 0.1)
         {
            xNonCollinearToAxis = 1.0;
            yNonCollinearToAxis = 0.0;
            zNonCollinearToAxis = 0.0;
         }
         else
         {
            xNonCollinearToAxis = 0.0;
            yNonCollinearToAxis = 1.0;
            zNonCollinearToAxis = 0.0;
         }

         double xOrthogonalToAxis = yNonCollinearToAxis * torus3DAxis.getZ() - zNonCollinearToAxis * torus3DAxis.getY();
         double yOrthogonalToAxis = zNonCollinearToAxis * torus3DAxis.getX() - xNonCollinearToAxis * torus3DAxis.getZ();
         double zOrthogonalToAxis = xNonCollinearToAxis * torus3DAxis.getY() - yNonCollinearToAxis * torus3DAxis.getX();
         double norm = Math.sqrt(EuclidCoreTools.normSquared(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis));

         double distanceFromTubeCenter = Math.sqrt(normSquared(percentageOnAxis, torus3DRadius));

         double scale = torus3DTubeRadius / distanceFromTubeCenter;
         double resultDistanceFromAxis = torus3DRadius - (torus3DRadius * scale);
         double resultPositionOnAxis = percentageOnAxis * scale;

         projectionToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         projectionToPack.scale(resultDistanceFromAxis / norm);
         projectionToPack.scaleAdd(resultPositionOnAxis, torus3DAxis, projectionToPack);
         projectionToPack.add(torus3DPosition);

         return true;
      }
      else
      {
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         if (EuclidCoreTools.normSquared(distanceFromAxis - torus3DRadius, percentageOnAxis) <= torus3DTubeRadius * torus3DTubeRadius)
            return false;

         double scale = torus3DRadius / distanceFromAxis;

         double xTubeCenter = xInPlane * scale + torus3DPosition.getX();
         double yTubeCenter = yInPlane * scale + torus3DPosition.getY();
         double zTubeCenter = zInPlane * scale + torus3DPosition.getZ();

         double distanceFromTubeCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(xTubeCenter, yTubeCenter, zTubeCenter, pointToProject);

         projectionToPack.set(pointToProject);
         projectionToPack.sub(xTubeCenter, yTubeCenter, zTubeCenter);
         projectionToPack.scale(torus3DTubeRadius / distanceFromTubeCenter);
         projectionToPack.add(xTubeCenter, yTubeCenter, zTubeCenter);

         return true;
      }
   }

   public static double doPoint3DTorus3DCollisionTest(Point3DReadOnly query, Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis,
                                                      double torus3DRadius, double torus3DTubeRadius, Point3DBasics closestPointToPack,
                                                      Vector3DBasics normalToPack)
   {
      double x = query.getX() - torus3DPosition.getX();
      double y = query.getY() - torus3DPosition.getY();
      double z = query.getZ() - torus3DPosition.getZ();
      double percentageOnAxis = TupleTools.dot(x, y, z, torus3DAxis);

      double xInPlane = x - percentageOnAxis * torus3DAxis.getX();
      double yInPlane = y - percentageOnAxis * torus3DAxis.getY();
      double zInPlane = z - percentageOnAxis * torus3DAxis.getZ();

      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(xInPlane, yInPlane, zInPlane);

      if (distanceSquaredFromAxis < TORUS_SMALLEST_DISTANCE_TO_AXIS)
      { // We need to setup a vector that is orthogonal to the torus' axis, then we'll perform the projection along that vector.
         double xNonCollinearToAxis, yNonCollinearToAxis, zNonCollinearToAxis;

         // Purposefully picking a large tolerance to ensure sanity of the cross-product.
         if (Math.abs(torus3DAxis.getY()) > 0.1 || Math.abs(torus3DAxis.getZ()) > 0.1)
         {
            xNonCollinearToAxis = 1.0;
            yNonCollinearToAxis = 0.0;
            zNonCollinearToAxis = 0.0;
         }
         else
         {
            xNonCollinearToAxis = 0.0;
            yNonCollinearToAxis = 1.0;
            zNonCollinearToAxis = 0.0;
         }

         double xOrthogonalToAxis = yNonCollinearToAxis * torus3DAxis.getZ() - zNonCollinearToAxis * torus3DAxis.getY();
         double yOrthogonalToAxis = zNonCollinearToAxis * torus3DAxis.getX() - xNonCollinearToAxis * torus3DAxis.getZ();
         double zOrthogonalToAxis = xNonCollinearToAxis * torus3DAxis.getY() - yNonCollinearToAxis * torus3DAxis.getX();
         double norm = Math.sqrt(EuclidCoreTools.normSquared(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis));

         double distanceFromTubeCenter = Math.sqrt(normSquared(percentageOnAxis, torus3DRadius));

         double scale = torus3DTubeRadius / distanceFromTubeCenter;
         double resultDistanceFromAxis = torus3DRadius - (torus3DRadius * scale);
         double resultPositionOnAxis = percentageOnAxis * scale;

         closestPointToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         closestPointToPack.scale(resultDistanceFromAxis / norm);
         closestPointToPack.scaleAdd(resultPositionOnAxis, torus3DAxis, closestPointToPack);
         closestPointToPack.add(torus3DPosition);

         normalToPack.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         normalToPack.scale(-torus3DRadius / norm);
         normalToPack.scaleAdd(percentageOnAxis, torus3DAxis, normalToPack);
         normalToPack.scale(1.0 / distanceFromTubeCenter);

         return distanceFromTubeCenter - torus3DTubeRadius;
      }
      else
      {
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         double scale = torus3DRadius / distanceFromAxis;

         double xTubeCenter = xInPlane * scale + torus3DPosition.getX();
         double yTubeCenter = yInPlane * scale + torus3DPosition.getY();
         double zTubeCenter = zInPlane * scale + torus3DPosition.getZ();

         double distanceFromTubeCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(xTubeCenter, yTubeCenter, zTubeCenter, query);

         normalToPack.set(query);
         normalToPack.sub(xTubeCenter, yTubeCenter, zTubeCenter);
         normalToPack.scale(1.0 / distanceFromTubeCenter);

         closestPointToPack.setAndScale(torus3DTubeRadius, normalToPack);
         closestPointToPack.add(xTubeCenter, yTubeCenter, zTubeCenter);

         return distanceFromTubeCenter - torus3DTubeRadius;
      }
   }

   public static double coneVolume(double height, double radius)
   {
      return Math.PI * radius * radius * height / 3.0;
   }

   public static double cylinderVolume(double length, double radius)
   {
      return Math.PI * radius * radius * length;
   }

   public static double icosahedronVolume(double edgeLength)
   {
      return edgeLength * edgeLength * edgeLength * (5.0 * (3.0 + Math.sqrt(5.0))) / 12.0;
   }

   public static double pyramidVolume(double height, double baseLength, double baseWidth)
   {
      return baseLength * baseWidth * height / 3.0;
   }

   public static double tetrahedronVolume(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, Point3DReadOnly d)
   {
      // From: http://mathworld.wolfram.com/Tetrahedron.html
      // V = 1/3 * | v1 . (v2 x v3)|
      // where the vectors v1, v2, and v3 are defined as follows:
      // v1 = b - a
      // v2 = c - a
      // v3 = d - a
      double x = (c.getY() - a.getY()) * (d.getZ() - a.getZ()) - (c.getZ() - a.getZ()) * (d.getY() - a.getY());
      double y = (c.getZ() - a.getZ()) * (d.getX() - a.getX()) - (c.getX() - a.getX()) * (d.getZ() - a.getZ());
      double z = (c.getX() - a.getX()) * (d.getY() - a.getY()) - (c.getY() - a.getY()) * (d.getX() - a.getX());
      return Math.abs(x * (b.getX() - a.getX()) + y * (b.getY() - a.getY()) + z * (b.getZ() - a.getZ())) / 6.0;
   }

   public static double icosahedronEdgeLength(double radius)
   {
      return radius / Math.sin(0.4 * Math.PI);
   }

   public static double icosahedronRadius(double edgeLength)
   {
      return edgeLength * Math.sin(0.4 * Math.PI);
   }
}
