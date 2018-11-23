package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreTools.*;
import static us.ihmc.euclid.tools.TupleTools.*;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeTools
{
   private static final double SPHERE_SMALLEST_DISTANCE_TO_ORIGIN = 1.0e-12;

   public static boolean isPoint3DInsideBox3D(Shape3DPoseReadOnly box3DPose, Vector3DReadOnly box3DSize, Point3DReadOnly query, double epsilon)
   {
      double dX = query.getX() - box3DPose.getTranslationX();
      double dY = query.getY() - box3DPose.getTranslationY();
      double dZ = query.getZ() - box3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, box3DPose.getXAxis());

      if (Math.abs(xLocalQuery) <= 0.5 * box3DSize.getX() + epsilon)
      {
         double yLocalQuery = dot(dX, dY, dZ, box3DPose.getYAxis());

         if (Math.abs(yLocalQuery) <= 0.5 * box3DSize.getY() + epsilon)
         {
            double zLocalQuery = dot(dX, dY, dZ, box3DPose.getZAxis());
            return Math.abs(zLocalQuery) <= 0.5 * box3DSize.getZ() + epsilon;
         }
      }
      return false;
   }

   public static double signedDistanceBetweenPoint3DAndBox3D(Shape3DPoseReadOnly box3DPose, Vector3DReadOnly box3DSize, Point3DReadOnly query)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double dx = query.getX() - box3DPose.getTranslationX();
      double dy = query.getY() - box3DPose.getTranslationY();
      double dz = query.getZ() - box3DPose.getTranslationZ();
      double xLocal = dot(dx, dy, dz, box3DPose.getXAxis());
      double yLocal = dot(dx, dy, dz, box3DPose.getYAxis());
      double zLocal = dot(dx, dy, dz, box3DPose.getZAxis());

      boolean isInside = Math.abs(xLocal) <= halfSizeX && Math.abs(yLocal) <= halfSizeY && Math.abs(zLocal) <= halfSizeZ;

      if (isInside)
      {
         dx = Math.abs(Math.abs(xLocal) - halfSizeX);
         dy = Math.abs(Math.abs(yLocal) - halfSizeY);
         dz = Math.abs(Math.abs(zLocal) - halfSizeZ);

         return -EuclidCoreTools.min(dx, dy, dz);
      }
      else
      {
         dx = xLocal - EuclidCoreTools.clamp(xLocal, halfSizeX);
         dy = yLocal - EuclidCoreTools.clamp(yLocal, halfSizeY);
         dz = zLocal - EuclidCoreTools.clamp(zLocal, halfSizeZ);

         return Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));
      }
   }

   public static boolean orthogonalProjectionOntoBox3D(Shape3DPoseReadOnly box3DPose, Vector3DReadOnly box3DSize, Point3DReadOnly pointToProject,
                                                       Point3DBasics projectionToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double dx = pointToProject.getX() - box3DPose.getTranslationX();
      double dy = pointToProject.getY() - box3DPose.getTranslationY();
      double dz = pointToProject.getZ() - box3DPose.getTranslationZ();

      double xLocal = dot(dx, dy, dz, box3DPose.getXAxis());
      double yLocal = dot(dx, dy, dz, box3DPose.getYAxis());
      double zLocal = dot(dx, dy, dz, box3DPose.getZAxis());

      if (Math.abs(xLocal) > halfSizeX || Math.abs(yLocal) > halfSizeY || Math.abs(zLocal) > halfSizeZ)
      {
         double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
         double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
         double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

         projectionToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);
         box3DPose.transform(projectionToPack);

         return true;
      }
      else
      {
         return false;
      }
   }

   public static double doPoint3DBox3DCollisionTest(Shape3DPoseReadOnly box3DPose, Vector3DReadOnly box3DSize, Point3DReadOnly query,
                                                    Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double halfSizeX = 0.5 * box3DSize.getX();
      double halfSizeY = 0.5 * box3DSize.getY();
      double halfSizeZ = 0.5 * box3DSize.getZ();

      double dx = query.getX() - box3DPose.getTranslationX();
      double dy = query.getY() - box3DPose.getTranslationY();
      double dz = query.getZ() - box3DPose.getTranslationZ();
      double xLocal = dot(dx, dy, dz, box3DPose.getXAxis());
      double yLocal = dot(dx, dy, dz, box3DPose.getYAxis());
      double zLocal = dot(dx, dy, dz, box3DPose.getZAxis());

      boolean isInside = Math.abs(xLocal) <= halfSizeX && Math.abs(yLocal) <= halfSizeY && Math.abs(zLocal) <= halfSizeZ;

      if (isInside)
      {
         dx = Math.abs(Math.abs(xLocal) - halfSizeX);
         dy = Math.abs(Math.abs(yLocal) - halfSizeY);
         dz = Math.abs(Math.abs(zLocal) - halfSizeZ);

         if (closestPointToPack != null)
         {
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

            box3DPose.transform(closestPointToPack);
         }

         if (normalToPack != null)
         {
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
            box3DPose.transform(normalToPack);
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

         if (closestPointToPack != null)
         {
            closestPointToPack.set(xLocalClamped, yLocalClamped, zLocalClamped);
            box3DPose.transform(closestPointToPack);
         }

         if (normalToPack != null)
         {
            normalToPack.set(dx, dy, dz);
            normalToPack.scale(1.0 / distance);
            box3DPose.transform(normalToPack);
         }

         return distance;
      }
   }

   public static boolean isPoint3DInsideCapsule3D(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength,
                                                  double capsule3DRadius, Point3DReadOnly query, double epsilon)
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

   public static double signedDistanceBetweenPoint3DAndCapsule3D(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength,
                                                                 double capsule3DRadius, Point3DReadOnly query)
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

   public static boolean orthogonalProjectionOntoCapsule3D(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength,
                                                           double capsule3DRadius, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
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

   public static double doPoint3DCapsule3DCollisionTest(Point3DReadOnly capsule3DPosition, Vector3DReadOnly capsule3DAxis, double capsule3DLength,
                                                        double capsule3DRadius, Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack,
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
         return doPoint3DSphere3DCollisionTest(capsule3DPosition, capsule3DRadius, query, closestPointOnSurfaceToPack, normalToPack);
      }

      double capsule3DHalfLength = 0.5 * capsule3DLength;

      double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(query, capsule3DPosition, capsule3DAxis);

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

   public static boolean isPoint3DInsideCylinder3D(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                                   double cylinder3DRadius, Point3DReadOnly query, double epsilon)
   {
      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(query.getX(), query.getY(), query.getZ(), cylinder3DPosition.getX(),
                                                                        cylinder3DPosition.getY(), cylinder3DPosition.getZ(), cylinder3DAxis.getX(),
                                                                        cylinder3DAxis.getY(), cylinder3DAxis.getZ());

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

   public static double signedDistanceBetweenPoint3DAndCylinder3D(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                                                  double cylinder3DRadius, Point3DReadOnly query)
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

   public static boolean orthogonalProjectionOntoCylinder3D(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                                            double cylinder3DRadius, Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
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

   public static double doPoint3DCylinder3DCollisionTest(Point3DReadOnly cylinder3DPosition, Vector3DReadOnly cylinder3DAxis, double cylinder3DLength,
                                                         double cylinder3DRadius, Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack,
                                                         Vector3DBasics normalToPack)
   {
      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         if (closestPointOnSurfaceToPack != null)
            closestPointOnSurfaceToPack.setToNaN();
         if (normalToPack != null)
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
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
            if (normalToPack != null)
               normalToPack.setAndNegate(cylinder3DAxis);
            return -(positionOnAxis + halfLength);
         }

         if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
            if (normalToPack != null)
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
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, query);
               if (normalToPack != null)
                  normalToPack.setAndNegate(cylinder3DAxis);
               return -(positionOnAxis + halfLength);
            }
            else
            { // Closer to the top face
               if (closestPointOnSurfaceToPack != null)
                  closestPointOnSurfaceToPack.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, query);
               if (normalToPack != null)
                  normalToPack.set(cylinder3DAxis);
               return positionOnAxis - halfLength;
            }
         }
         else
         { // Closer to the cylinder part
            double directionToQueryX = axisToQueryX / distanceFromAxis;
            double directionToQueryY = axisToQueryY / distanceFromAxis;
            double directionToQueryZ = axisToQueryZ / distanceFromAxis;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(directionToQueryX, directionToQueryY, directionToQueryZ);
               closestPointOnSurfaceToPack.scale(cylinder3DRadius);
               closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            }

            if (normalToPack != null)
            {
               normalToPack.set(directionToQueryX, directionToQueryY, directionToQueryZ);
            }
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

            if (closestPointOnSurfaceToPack != null)
               closestPointOnSurfaceToPack.set(closestX, closestY, closestZ);

            if (normalToPack != null)
            {
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
            }

            return distance;
         }
         else
         { // Closest point is on the cylinder.
            double directionToQueryX = axisToQueryX / distanceFromAxis;
            double directionToQueryY = axisToQueryY / distanceFromAxis;
            double directionToQueryZ = axisToQueryZ / distanceFromAxis;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(directionToQueryX, directionToQueryY, directionToQueryZ);
               closestPointOnSurfaceToPack.scale(cylinder3DRadius);
               closestPointOnSurfaceToPack.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            }

            if (normalToPack != null)
            {
               normalToPack.set(directionToQueryX, directionToQueryY, directionToQueryZ);
            }

            return distanceFromAxis - cylinder3DRadius;
         }
      }
   }

   public static boolean isPoint3DInsideEllipsoid3D(Shape3DPoseReadOnly ellipsoid3DPose, Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly query,
                                                    double epsilon)
   {
      double dX = query.getX() - ellipsoid3DPose.getTranslationX();
      double dY = query.getY() - ellipsoid3DPose.getTranslationY();
      double dZ = query.getZ() - ellipsoid3DPose.getTranslationZ();

      double xLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getZAxis());

      double scaledX = xLocalQuery / (ellipsoid3DRadii.getX() + epsilon);
      double scaledY = yLocalQuery / (ellipsoid3DRadii.getY() + epsilon);
      double scaledZ = zLocalQuery / (ellipsoid3DRadii.getZ() + epsilon);

      return EuclidCoreTools.normSquared(scaledX, scaledY, scaledZ) <= 1.0;
   }

   public static double signedDistanceBetweenPoint3DAndEllipsoid3D(Shape3DPoseReadOnly ellipsoid3DPose, Vector3DReadOnly ellipsoid3DRadii,
                                                                   Point3DReadOnly query)
   {
      return doPoint3DEllipsoid3DCollisionTest(ellipsoid3DPose, ellipsoid3DRadii, query, null, null);
   }

   public static boolean orthogonalProjectionOntoEllipsoid3D(Shape3DPoseReadOnly ellipsoid3DPose, Vector3DReadOnly ellipsoid3DRadii,
                                                             Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double dX = pointToProject.getX() - ellipsoid3DPose.getTranslationX();
      double dY = pointToProject.getY() - ellipsoid3DPose.getTranslationY();
      double dZ = pointToProject.getZ() - ellipsoid3DPose.getTranslationZ();

      double xLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getZAxis());

      double sumOfSquares = EuclidCoreTools.normSquared(xLocalQuery / xRadius, yLocalQuery / yRadius, zLocalQuery / zRadius);

      if (sumOfSquares > 1.0)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);
         projectionToPack.sub(pointToProject, ellipsoid3DPose.getShapePosition());
         projectionToPack.scale(scaleFactor);
         projectionToPack.add(ellipsoid3DPose.getShapePosition());
         return true;
      }
      else
      {
         return false;
      }
   }

   public static double doPoint3DEllipsoid3DCollisionTest(Shape3DPoseReadOnly ellipsoid3DPose, Vector3DReadOnly ellipsoid3DRadii, Point3DReadOnly query,
                                                          Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double dX = query.getX() - ellipsoid3DPose.getTranslationX();
      double dY = query.getY() - ellipsoid3DPose.getTranslationY();
      double dZ = query.getZ() - ellipsoid3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getZAxis());

      double sumOfSquares = EuclidCoreTools.normSquared(xLocalQuery / xRadius, yLocalQuery / yRadius, zLocalQuery / zRadius);

      if (sumOfSquares > 1.0e-10)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

         if (closestPointToPack != null)
         {
            closestPointToPack.sub(query, ellipsoid3DPose.getShapePosition());
            closestPointToPack.scale(scaleFactor);
            closestPointToPack.add(ellipsoid3DPose.getShapePosition());
         }

         if (normalToPack != null)
         {
            double xScale = 1.0 / (xRadius * xRadius);
            double yScale = 1.0 / (yRadius * yRadius);
            double zScale = 1.0 / (zRadius * zRadius);

            normalToPack.set(xLocalQuery, yLocalQuery, zLocalQuery);
            normalToPack.scale(xScale, yScale, zScale);
            normalToPack.normalize();
            ellipsoid3DPose.transform(normalToPack);
         }

         return query.distanceFromOrigin() * (1.0 - scaleFactor);
      }
      else
      {
         if (closestPointToPack != null)
         {
            closestPointToPack.scaleAdd(zRadius, ellipsoid3DPose.getZAxis(), ellipsoid3DPose.getShapePosition());
         }

         if (normalToPack != null)
         {
            normalToPack.set(ellipsoid3DPose.getZAxis());
         }

         return zLocalQuery - zRadius;
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

   public static boolean isPoint3DInsideRamp3D(Shape3DPoseReadOnly ramp3DPose, Vector3DReadOnly ramp3DSize, Point3DReadOnly query, double epsilon)
   {
      double dX = query.getX() - ramp3DPose.getTranslationX();
      double dY = query.getY() - ramp3DPose.getTranslationY();
      double dZ = query.getZ() - ramp3DPose.getTranslationZ();

      double zLocalQuery = dot(dX, dY, dZ, ramp3DPose.getZAxis());
      if (zLocalQuery < -epsilon)
         return false;

      double xLocalQuery = dot(dX, dY, dZ, ramp3DPose.getXAxis());
      if (xLocalQuery > ramp3DSize.getX() + epsilon)
         return false;

      double halfWidth = 0.5 * ramp3DSize.getY() + epsilon;

      double yLocalQuery = dot(dX, dY, dZ, ramp3DPose.getYAxis());
      if (yLocalQuery < -halfWidth || yLocalQuery > halfWidth)
         return false;

      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;

      // Computing the signed distance between the query and the slope face, negative value means the query is below the slope.
      return rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ <= epsilon;
   }

   public static double signedDistanceBetweenPoint3DAndRamp3D(Shape3DPoseReadOnly ramp3DPose, Vector3DReadOnly ramp3DSize, Point3DReadOnly query)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double dX = query.getX() - ramp3DPose.getTranslationX();
      double dY = query.getY() - ramp3DPose.getTranslationY();
      double dZ = query.getZ() - ramp3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, ramp3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ramp3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ramp3DPose.getZAxis());

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         if (xClosest == xLocalQuery && yClosest == yLocalQuery)
            return -zLocalQuery;
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest);
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         if (yClosest == yLocalQuery && zClosest == zLocalQuery)
            return xLocalQuery - ramp3DSize.getX();
         else
            return EuclidGeometryTools.distanceBetweenPoint3Ds(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest);
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         return EuclidGeometryTools.distanceBetweenPoint3Ds(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest);
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            return Math.abs(yLocalQuery) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            return EuclidGeometryTools.distanceBetweenPoint3Ds(xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
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
            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            return -distanceToSlopeFace;
         }
      }
   }

   public static boolean orthogonalProjectionOntoRamp3D(Shape3DPoseReadOnly ramp3DPose, Vector3DReadOnly ramp3DSize, Point3DReadOnly pointToProject,
                                                        Point3DBasics projectionToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double dX = pointToProject.getX() - ramp3DPose.getTranslationX();
      double dY = pointToProject.getY() - ramp3DPose.getTranslationY();
      double dZ = pointToProject.getZ() - ramp3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, ramp3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ramp3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ramp3DPose.getZAxis());

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(projectionToPack);
         return true;
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         projectionToPack.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(projectionToPack);
         return true;
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         projectionToPack.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(projectionToPack);
         return true;
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xLocalQuery;
            double zClosest = zLocalQuery;

            projectionToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(projectionToPack);
            return true;
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            projectionToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(projectionToPack);
            return true;
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yLocalQuery;
         double zClosest = dot * rampDirectionZ;

         projectionToPack.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(projectionToPack);
         return true;
      }
      else
      { // Query is inside the ramp
         return false;
      }
   }

   public static double doPoint3DRamp3DCollisionTest(Shape3DPoseReadOnly ramp3DPose, Vector3DReadOnly ramp3DSize, Point3DReadOnly query,
                                                     Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalToPack)
   {
      double rampLength = computeRamp3DLength(ramp3DSize);
      double rampDirectionX = ramp3DSize.getX() / rampLength;
      double rampDirectionZ = ramp3DSize.getZ() / rampLength;
      double rampNormalX = -rampDirectionZ;
      double rampNormalZ = rampDirectionX;

      double dX = query.getX() - ramp3DPose.getTranslationX();
      double dY = query.getY() - ramp3DPose.getTranslationY();
      double dZ = query.getZ() - ramp3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, ramp3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ramp3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ramp3DPose.getZAxis());

      double halfWidth = 0.5 * ramp3DSize.getY();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(closestPointOnSurfaceToPack);
         }

         if (xClosest == xLocalQuery && yClosest == yLocalQuery)
         {
            if (normalToPack != null)
            {
               normalToPack.setAndNegate(ramp3DPose.getZAxis());
            }

            return -zLocalQuery;
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(ramp3DPose, xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(closestPointOnSurfaceToPack);
         }

         if (yClosest == yLocalQuery && zClosest == zLocalQuery)
         {
            if (normalToPack != null)
            {
               normalToPack.set(ramp3DPose.getXAxis());
            }

            return xLocalQuery - ramp3DSize.getX();
         }
         else
         {
            return computeNormalAndDistanceFromClosestPoint(ramp3DPose, xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(closestPointOnSurfaceToPack);
         }

         return computeNormalAndDistanceFromClosestPoint(ramp3DPose, xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xLocalQuery;
            double zClosest = zLocalQuery;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               if (yLocalQuery >= 0.0)
                  normalToPack.set(ramp3DPose.getYAxis());
               else
                  normalToPack.setAndNegate(ramp3DPose.getYAxis());
            }

            return Math.abs(yLocalQuery) - halfWidth;
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            return computeNormalAndDistanceFromClosestPoint(ramp3DPose, xLocalQuery, yLocalQuery, zLocalQuery, xClosest, yClosest, zClosest, normalToPack);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yLocalQuery;
         double zClosest = dot * rampDirectionZ;

         if (closestPointOnSurfaceToPack != null)
         {
            closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(closestPointOnSurfaceToPack);
         }

         if (normalToPack != null)
         {
            normalToPack.set(rampNormalX, 0.0, rampNormalZ);
            ramp3DPose.transform(normalToPack);
         }

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
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xLocalQuery, -halfWidth, zLocalQuery);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               normalToPack.setAndNegate(ramp3DPose.getYAxis());
            }

            return -distanceToRightFace;
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xLocalQuery, halfWidth, zLocalQuery);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               normalToPack.set(ramp3DPose.getYAxis());
            }

            return -distanceToLeftFace;
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(ramp3DSize.getX(), yLocalQuery, zLocalQuery);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               normalToPack.set(ramp3DPose.getXAxis());
            }

            return -distanceToRearFace;
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            if (closestPointOnSurfaceToPack != null)
            {
               closestPointOnSurfaceToPack.set(xLocalQuery, yLocalQuery, 0.0);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               normalToPack.setAndNegate(ramp3DPose.getZAxis());
            }

            return -distanceToBottomFace;
         }
         else
         { // Query is closer to the slope face
            if (closestPointOnSurfaceToPack != null)
            {
               double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
               double xClosest = dot * rampDirectionX;
               double yClosest = yLocalQuery;
               double zClosest = dot * rampDirectionZ;

               closestPointOnSurfaceToPack.set(xClosest, yClosest, zClosest);
               ramp3DPose.transform(closestPointOnSurfaceToPack);
            }

            if (normalToPack != null)
            {
               normalToPack.set(rampNormalX, 0.0, rampNormalZ);
               ramp3DPose.transform(normalToPack);
            }

            return -distanceToSlopeFace;
         }
      }
   }

   private static double computeNormalAndDistanceFromClosestPoint(RigidBodyTransformReadOnly transformToWorld, double xLocalQuery, double yLocalQuery,
                                                                  double zLocalQuery, double xLocalClosest, double yLocalClosest, double zLocalClosest,
                                                                  Vector3DBasics normalToPack)
   {
      double dx = xLocalQuery - xLocalClosest;
      double dy = yLocalQuery - yLocalClosest;
      double dz = zLocalQuery - zLocalClosest;

      double distance = Math.sqrt(EuclidCoreTools.normSquared(dx, dy, dz));

      if (normalToPack != null)
      {
         normalToPack.set(dx, dy, dz);
         normalToPack.scale(1.0 / distance);
         transformToWorld.transform(normalToPack);
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

   public static boolean isPoint3DInsideSphere3D(Point3DReadOnly spherePosition, double sphereRadius, Point3DReadOnly query, double epsilon)
   {
      double radiusWithEpsilon = sphereRadius + epsilon;
      return spherePosition.distanceSquared(query) <= radiusWithEpsilon * radiusWithEpsilon;
   }

   public static double signedDistanceBetweenPoint3DAndSphere3D(Point3DReadOnly spherePosition, double sphereRadius, Point3DReadOnly query)
   {
      return spherePosition.distance(query) - sphereRadius;
   }

   public static boolean orthogonalProjectionOntoSphere3D(Point3DReadOnly spherePosition, double sphereRadius, Point3DReadOnly pointToProject,
                                                          Point3DBasics projectionToPack)
   {
      double distanceSquared = spherePosition.distanceSquared(pointToProject);
      if (distanceSquared <= sphereRadius * sphereRadius)
         return false;

      projectionToPack.sub(pointToProject, spherePosition);
      projectionToPack.scale(sphereRadius / Math.sqrt(distanceSquared));
      projectionToPack.add(spherePosition);
      return true;
   }

   public static double doPoint3DSphere3DCollisionTest(Point3DReadOnly spherePosition, double sphereRadius, Point3DReadOnly query,
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

   public static double doSphere3DSphere3DCollisionTest(Point3DReadOnly firstSpherePosition, double firstSphereRadius, Point3DReadOnly secondSpherePosition,
                                                        double secondSphereRadius, Point3DBasics firstClosestPointToPack,
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

   public static double doSphere3DCylinder3DCollisionTest(Point3DReadOnly spherePosition, double sphereRadius, Point3DReadOnly cylinderPosition,
                                                          Vector3DReadOnly cylinderAxis, double cylinderLength, double cylinderRadius,
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

   public static boolean isPoint3DInsideTorus3D(Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis, double torus3DRadius, double torus3DTubeRadius,
                                                Point3DReadOnly query, double epsilon)
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

   public static double signedDistanceBetweenPoint3DAndTorus3D(Point3DReadOnly torus3DPosition, Vector3DReadOnly torus3DAxis, double torus3DRadius,
                                                               double torus3DTubeRadius, Point3DReadOnly query)
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

   public static boolean orthogonalProjectionOntoTorus3D(Shape3DPoseReadOnly torus3DPose, double torus3DRadius, double torus3DTubeRadius,
                                                         Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      double dx = pointToProject.getX() - torus3DPose.getTranslationX();
      double dy = pointToProject.getY() - torus3DPose.getTranslationY();
      double dz = pointToProject.getZ() - torus3DPose.getTranslationZ();

      double xLocal = dot(dx, dy, dz, torus3DPose.getXAxis());
      double yLocal = dot(dx, dy, dz, torus3DPose.getYAxis());
      double zLocal = dot(dx, dy, dz, torus3DPose.getZAxis());

      double xyLengthSquared = normSquared(xLocal, yLocal);

      if (xyLengthSquared < 1.0e-12)
      {
         double xzLength = Math.sqrt(normSquared(torus3DRadius, zLocal));

         double scale = torus3DTubeRadius / xzLength;
         double closestTubeCenterX = torus3DRadius;
         double closestTubeCenterY = 0.0;

         double tubeCenterToSurfaceX = -torus3DRadius * scale;
         double tubeCenterToSurfaceY = 0.0;
         double tubeCenterToSurfaceZ = zLocal * scale;

         projectionToPack.set(closestTubeCenterX, closestTubeCenterY, 0.0);
         projectionToPack.add(tubeCenterToSurfaceX, tubeCenterToSurfaceY, tubeCenterToSurfaceZ);
         return true;
      }
      else
      {
         double xyLength = Math.sqrt(xyLengthSquared);

         if (EuclidCoreTools.normSquared(xyLength - torus3DRadius, zLocal) <= torus3DTubeRadius * torus3DTubeRadius)
            return false;

         double xyScale = torus3DRadius / xyLength;

         double closestTubeCenterX = xLocal * xyScale;
         double closestTubeCenterY = yLocal * xyScale;

         dx = xLocal - closestTubeCenterX;
         dy = yLocal - closestTubeCenterY;
         dz = zLocal;

         double distance = Math.sqrt(normSquared(dx, dy, dz));

         projectionToPack.set(dx, dy, dz);
         projectionToPack.scale(torus3DTubeRadius / distance);
         projectionToPack.add(closestTubeCenterX, closestTubeCenterY, 0.0);
         return true;
      }
   }

   public static double doPoint3DTorus3DCollisionTest(Shape3DPoseReadOnly torus3DPose, double torus3DRadius, double torus3DTubeRadius, Point3DReadOnly query,
                                                      Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      double dx = query.getX() - torus3DPose.getTranslationX();
      double dy = query.getY() - torus3DPose.getTranslationY();
      double dz = query.getZ() - torus3DPose.getTranslationZ();

      double xLocal = dot(dx, dy, dz, torus3DPose.getXAxis());
      double yLocal = dot(dx, dy, dz, torus3DPose.getYAxis());
      double zLocal = dot(dx, dy, dz, torus3DPose.getZAxis());

      double xyLengthSquared = normSquared(xLocal, yLocal);

      if (xyLengthSquared < 1.0e-12)
      {
         double xzLength = Math.sqrt(normSquared(torus3DRadius, zLocal));

         if (closestPointToPack != null)
         {
            double scale = torus3DTubeRadius / xzLength;
            double closestTubeCenterX = torus3DRadius;
            double closestTubeCenterY = 0.0;

            double tubeCenterToSurfaceX = -torus3DRadius * scale;
            double tubeCenterToSurfaceY = 0.0;
            double tubeCenterToSurfaceZ = zLocal * scale;

            closestPointToPack.set(closestTubeCenterX, closestTubeCenterY, 0.0);
            closestPointToPack.add(tubeCenterToSurfaceX, tubeCenterToSurfaceY, tubeCenterToSurfaceZ);
         }

         if (normalToPack != null)
         {
            normalToPack.set(-torus3DRadius, 0.0, zLocal);
            normalToPack.scale(1.0 / xzLength);
         }

         return xzLength - torus3DTubeRadius;
      }
      else
      {
         double xyScale = torus3DRadius / Math.sqrt(xyLengthSquared);

         double closestTubeCenterX = xLocal * xyScale;
         double closestTubeCenterY = yLocal * xyScale;

         dx = xLocal - closestTubeCenterX;
         dy = yLocal - closestTubeCenterY;
         dz = zLocal;

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
}
