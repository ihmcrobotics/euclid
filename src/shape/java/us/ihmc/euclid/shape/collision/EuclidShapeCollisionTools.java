package us.ihmc.euclid.shape.collision;

import static us.ihmc.euclid.shape.tools.EuclidShapeTools.*;
import static us.ihmc.euclid.tools.EuclidCoreTools.*;
import static us.ihmc.euclid.tools.TupleTools.*;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeCollisionTools
{
   private static final double SPHERE_SMALLEST_DISTANCE_TO_ORIGIN = 1.0e-12;
   private static final double TORUS_SMALLEST_DISTANCE_TO_AXIS = 1.0e-12;

   public static void doPointShape3DBox3DCollisionTest(PointShape3DReadOnly pointShape3D, Box3DReadOnly box3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DBox3DCollisionTest(pointShape3D, box3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(box3D);
   }

   public static void doSphere3DBox3DCollisionTest(Sphere3DReadOnly sphere3D, Box3DReadOnly box3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DBox3DCollisionTest(sphere3D.getPosition(), box3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(box3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DBox3DCollisionTest(Point3DReadOnly point3D, Box3DReadOnly box3D, Shape3DCollisionTestResult resultToPack)
   {
      resultToPack.setToNaN();

      double halfSizeX = 0.5 * box3D.getSizeX();
      double halfSizeY = 0.5 * box3D.getSizeY();
      double halfSizeZ = 0.5 * box3D.getSizeZ();

      double dx = point3D.getX() - box3D.getPose().getTranslationX();
      double dy = point3D.getY() - box3D.getPose().getTranslationY();
      double dz = point3D.getZ() - box3D.getPose().getTranslationZ();
      double xLocal = dot(dx, dy, dz, box3D.getPose().getXAxis());
      double yLocal = dot(dx, dy, dz, box3D.getPose().getYAxis());
      double zLocal = dot(dx, dy, dz, box3D.getPose().getZAxis());

      boolean isInside = Math.abs(xLocal) <= halfSizeX && Math.abs(yLocal) <= halfSizeY && Math.abs(zLocal) <= halfSizeZ;
      resultToPack.setShapesAreColliding(isInside);

      if (isInside)
      {

         dx = Math.abs(Math.abs(xLocal) - halfSizeX);
         dy = Math.abs(Math.abs(yLocal) - halfSizeY);
         dz = Math.abs(Math.abs(zLocal) - halfSizeZ);

         Point3D pointOnB = resultToPack.getPointOnB();
         pointOnB.set(xLocal, yLocal, zLocal);

         Vector3D normalOnB = resultToPack.getNormalOnB();
         normalOnB.setToZero();

         if (dx < dy)
         {
            if (dx < dz)
            {
               pointOnB.setX(Math.copySign(halfSizeX, xLocal));
               normalOnB.setX(Math.copySign(1.0, xLocal));
            }
            else
            {
               pointOnB.setZ(Math.copySign(halfSizeZ, zLocal));
               normalOnB.setZ(Math.copySign(1.0, zLocal));
            }
         }
         else
         {
            if (dy < dz)
            {
               pointOnB.setY(Math.copySign(halfSizeY, yLocal));
               normalOnB.setY(Math.copySign(1.0, yLocal));
            }
            else
            {
               pointOnB.setZ(Math.copySign(halfSizeZ, zLocal));
               normalOnB.setZ(Math.copySign(1.0, zLocal));
            }
         }

         box3D.getPose().transform(pointOnB);
         box3D.getPose().transform(normalOnB);

         resultToPack.setDepth(EuclidCoreTools.min(dx, dy, dz));
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

         Point3D pointOnB = resultToPack.getPointOnB();
         pointOnB.set(xLocalClamped, yLocalClamped, zLocalClamped);
         box3D.getPose().transform(pointOnB);

         Vector3D normalOnB = resultToPack.getNormalOnB();
         normalOnB.set(dx, dy, dz);
         normalOnB.scale(1.0 / distance);
         box3D.getPose().transform(normalOnB);

         resultToPack.setDistance(distance);
      }

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
   }

   public static void doPointShape3DCapsule3DCollisionTest(PointShape3DReadOnly pointShape3D, Capsule3DReadOnly capsule3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DCapsule3DCollisionTest(pointShape3D, capsule3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(capsule3D);
   }

   public static void doCapsule3DCapsule3DCollisionTest(Capsule3DReadOnly capsule3DA, Capsule3DReadOnly capsule3DB, Shape3DCollisionTestResult resultToPack)
   {
      Point3D pointOnA = resultToPack.getPointOnA();
      Point3D pointOnB = resultToPack.getPointOnB();
      double distanceBetweenAxes = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(capsule3DA.getTopCenter(), capsule3DA.getBottomCenter(),
                                                                                               capsule3DB.getTopCenter(), capsule3DB.getBottomCenter(),
                                                                                               pointOnA, pointOnB);

      Vector3D normalOnA = resultToPack.getNormalOnA();
      Vector3D normalOnB = resultToPack.getNormalOnB();
      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distanceBetweenAxes);
      normalOnB.setAndNegate(normalOnA);

      pointOnA.scaleAdd(capsule3DA.getRadius(), normalOnA, pointOnA);
      pointOnB.scaleAdd(capsule3DB.getRadius(), normalOnB, pointOnB);

      double distance = distanceBetweenAxes - capsule3DA.getRadius() - capsule3DB.getRadius();

      if (distance < 0.0)
      {
         resultToPack.setShapesAreColliding(true);
         resultToPack.setDepth(-distance);
      }
      else
      {
         resultToPack.setShapesAreColliding(false);
         resultToPack.setDistance(distance);
      }

      resultToPack.setShapeA(capsule3DA);
      resultToPack.setShapeB(capsule3DB);
   }

   public static void doSphere3DBox3DCollisionTest(Sphere3DReadOnly sphere3D, Capsule3DReadOnly capsule3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DCapsule3DCollisionTest(sphere3D.getPosition(), capsule3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(capsule3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DCapsule3DCollisionTest(Point3DReadOnly point3D, Capsule3DReadOnly capsule3D, Shape3DCollisionTestResult resultToPack)
   {
      Point3DReadOnly capsule3DPosition = capsule3D.getPosition();
      Vector3DReadOnly capsule3DAxis = capsule3D.getAxis();
      double capsule3DLength = capsule3D.getLength();
      double capsule3DRadius = capsule3D.getRadius();

      if (capsule3DRadius <= 0.0 || capsule3DLength < 0.0)
      {
         resultToPack.setToNaN();
      }
      else if (capsule3DLength == 0.0)
      {
         doPoint3DSphere3DCollisionTest(point3D, capsule3DPosition, capsule3DRadius, resultToPack);
      }
      else
      {
         double capsule3DHalfLength = 0.5 * capsule3DLength;
         double percentageOnAxis = EuclidGeometryTools.percentageAlongLine3D(point3D, capsule3DPosition, capsule3DAxis);

         if (Math.abs(percentageOnAxis) < capsule3DHalfLength)
         {
            resultToPack.setToNaN();
            double projectionOnAxisX = capsule3DPosition.getX() + percentageOnAxis * capsule3DAxis.getX();
            double projectionOnAxisY = capsule3DPosition.getY() + percentageOnAxis * capsule3DAxis.getY();
            double projectionOnAxisZ = capsule3DPosition.getZ() + percentageOnAxis * capsule3DAxis.getZ();
            double distanceFromAxis = EuclidGeometryTools.distanceBetweenPoint3Ds(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ, point3D);

            Point3D pointOnB = resultToPack.getPointOnB();
            pointOnB.set(point3D);
            pointOnB.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            pointOnB.scale(capsule3DRadius / distanceFromAxis);
            pointOnB.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

            Vector3D normalOnB = resultToPack.getNormalOnB();
            normalOnB.set(point3D);
            normalOnB.sub(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            normalOnB.scale(1.0 / distanceFromAxis);

            double signedDistance = distanceFromAxis - capsule3DRadius;
            if (signedDistance < 0.0)
            {
               resultToPack.setShapesAreColliding(true);
               resultToPack.setDepth(-signedDistance);
            }
            else
            {
               resultToPack.setShapesAreColliding(false);
               resultToPack.setDistance(signedDistance);
            }
            resultToPack.getPointOnA().set(point3D);
            resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
         }
         else if (percentageOnAxis > 0.0)
         {
            double topCenterX = capsule3DPosition.getX() + capsule3DHalfLength * capsule3DAxis.getX();
            double topCenterY = capsule3DPosition.getY() + capsule3DHalfLength * capsule3DAxis.getY();
            double topCenterZ = capsule3DPosition.getZ() + capsule3DHalfLength * capsule3DAxis.getZ();
            doPoint3DSphere3DCollisionTest(point3D, capsule3DRadius, topCenterX, topCenterY, topCenterZ, resultToPack);
         }
         else // if (percentageOnAxis < 0.0)
         {
            double bottomCenterX = capsule3DPosition.getX() - capsule3DHalfLength * capsule3DAxis.getX();
            double bottomCenterY = capsule3DPosition.getY() - capsule3DHalfLength * capsule3DAxis.getY();
            double bottomCenterZ = capsule3DPosition.getZ() - capsule3DHalfLength * capsule3DAxis.getZ();
            doPoint3DSphere3DCollisionTest(point3D, capsule3DRadius, bottomCenterX, bottomCenterY, bottomCenterZ, resultToPack);
         }
      }
   }

   public static void doPointShape3DCylinder3DCollisionTest(PointShape3DReadOnly pointShape3D, Cylinder3DReadOnly cylinder3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DCylinder3DCollisionTest(pointShape3D, cylinder3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(cylinder3D);
   }

   public static void doSphere3DCylinder3DCollisionTest(Sphere3DReadOnly sphere3D, Cylinder3DReadOnly cylinder3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DCylinder3DCollisionTest(sphere3D.getPosition(), cylinder3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(cylinder3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DCylinder3DCollisionTest(Point3DReadOnly point3D, Cylinder3DReadOnly cylinder3D, Shape3DCollisionTestResult resultToPack)
   {
      Point3DReadOnly cylinder3DPosition = cylinder3D.getPosition();
      Vector3DReadOnly cylinder3DAxis = cylinder3D.getAxis();
      double cylinder3DLength = cylinder3D.getLength();
      double cylinder3DRadius = cylinder3D.getRadius();

      if (cylinder3DRadius <= 0.0 || cylinder3DLength <= 0.0)
      {
         resultToPack.setToNaN();
      }

      double positionOnAxis = EuclidGeometryTools.percentageAlongLine3D(point3D, cylinder3DPosition, cylinder3DAxis);

      double projectionOnAxisX = cylinder3DPosition.getX() + positionOnAxis * cylinder3DAxis.getX();
      double projectionOnAxisY = cylinder3DPosition.getY() + positionOnAxis * cylinder3DAxis.getY();
      double projectionOnAxisZ = cylinder3DPosition.getZ() + positionOnAxis * cylinder3DAxis.getZ();

      double axisToQueryX = point3D.getX() - projectionOnAxisX;
      double axisToQueryY = point3D.getY() - projectionOnAxisY;
      double axisToQueryZ = point3D.getZ() - projectionOnAxisZ;
      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(axisToQueryX, axisToQueryY, axisToQueryZ);

      double halfLength = 0.5 * cylinder3DLength;

      Point3D pointOnB = resultToPack.getPointOnB();
      Vector3D normalOnB = resultToPack.getNormalOnB();

      if (distanceSquaredFromAxis <= cylinder3DRadius * cylinder3DRadius)
      {
         if (positionOnAxis < -halfLength)
         { // The query is directly below the cylinder
            pointOnB.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, point3D);
            normalOnB.setAndNegate(cylinder3DAxis);
            resultToPack.setShapesAreColliding(false);
            resultToPack.setDistance(-positionOnAxis - halfLength);
         }
         else if (positionOnAxis > halfLength)
         { // The query is directly above the cylinder
            pointOnB.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, point3D);
            normalOnB.set(cylinder3DAxis);
            resultToPack.setShapesAreColliding(false);
            resultToPack.setDistance(positionOnAxis - halfLength);
         }
         else
         {
            // The query is inside the cylinder
            double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);
            double dh = halfLength - Math.abs(positionOnAxis);
            double dr = cylinder3DRadius - distanceFromAxis;
            resultToPack.setShapesAreColliding(true);

            if (dh < dr)
            {
               if (positionOnAxis < 0)
               { // Closer to the bottom face
                  pointOnB.scaleAdd(-positionOnAxis - halfLength, cylinder3DAxis, point3D);
                  normalOnB.setAndNegate(cylinder3DAxis);
                  resultToPack.setDepth(positionOnAxis + halfLength);
               }
               else
               { // Closer to the top face
                  pointOnB.scaleAdd(-positionOnAxis + halfLength, cylinder3DAxis, point3D);
                  normalOnB.set(cylinder3DAxis);
                  resultToPack.setDepth(-positionOnAxis + halfLength);
               }
            }
            else
            { // Closer to the cylinder part
               double directionToQueryX = axisToQueryX / distanceFromAxis;
               double directionToQueryY = axisToQueryY / distanceFromAxis;
               double directionToQueryZ = axisToQueryZ / distanceFromAxis;

               pointOnB.set(directionToQueryX, directionToQueryY, directionToQueryZ);
               pointOnB.scale(cylinder3DRadius);
               pointOnB.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);

               normalOnB.set(directionToQueryX, directionToQueryY, directionToQueryZ);
               resultToPack.setDepth(cylinder3DRadius - distanceFromAxis);
            }
         }
      }
      else
      { // The query is outside and closest to the cylinder's side.
         resultToPack.setShapesAreColliding(false);
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
            double dX = point3D.getX() - closestX;
            double dY = point3D.getY() - closestY;
            double dZ = point3D.getZ() - closestZ;
            double distance = Math.sqrt(EuclidCoreTools.normSquared(dX, dY, dZ));

            pointOnB.set(closestX, closestY, closestZ);

            if (distance < 1.0e-12)
            {
               if (positionOnAxis > 0.0)
                  normalOnB.set(cylinder3DAxis);
               else
                  normalOnB.setAndNegate(cylinder3DAxis);
            }
            else
            {
               normalOnB.set(dX, dY, dZ);
               normalOnB.scale(1.0 / distance);
            }

            resultToPack.setDistance(distance);
         }
         else
         { // Closest point is on the cylinder.
            double directionToQueryX = axisToQueryX / distanceFromAxis;
            double directionToQueryY = axisToQueryY / distanceFromAxis;
            double directionToQueryZ = axisToQueryZ / distanceFromAxis;

            pointOnB.set(directionToQueryX, directionToQueryY, directionToQueryZ);
            pointOnB.scale(cylinder3DRadius);
            pointOnB.add(projectionOnAxisX, projectionOnAxisY, projectionOnAxisZ);
            normalOnB.set(directionToQueryX, directionToQueryY, directionToQueryZ);

            resultToPack.setDistance(distanceFromAxis - cylinder3DRadius);
         }
      }

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
   }

   public static void doPointShape3DEllipsoid3DCollisionTest(PointShape3DReadOnly pointShape3D, Ellipsoid3DReadOnly ellipsoid3D,
                                                             Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DEllipsoid3DCollisionTest(pointShape3D, ellipsoid3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ellipsoid3D);
   }

   public static void doSphere3DEllipsoid3DCollisionTest(Sphere3DReadOnly sphere3D, Ellipsoid3DReadOnly ellipsoid3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DEllipsoid3DCollisionTest(sphere3D.getPosition(), ellipsoid3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ellipsoid3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DEllipsoid3DCollisionTest(Point3DReadOnly point3D, Ellipsoid3DReadOnly ellipsoid3D, Shape3DCollisionTestResult resultToPack)
   {
      Shape3DPoseReadOnly ellipsoid3DPose = ellipsoid3D.getPose();
      Vector3DReadOnly ellipsoid3DRadii = ellipsoid3D.getRadii();

      double xRadius = ellipsoid3DRadii.getX();
      double yRadius = ellipsoid3DRadii.getY();
      double zRadius = ellipsoid3DRadii.getZ();

      double dX = point3D.getX() - ellipsoid3DPose.getTranslationX();
      double dY = point3D.getY() - ellipsoid3DPose.getTranslationY();
      double dZ = point3D.getZ() - ellipsoid3DPose.getTranslationZ();
      double xLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getXAxis());
      double yLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getYAxis());
      double zLocalQuery = dot(dX, dY, dZ, ellipsoid3DPose.getZAxis());
      Point3D pointOnB = resultToPack.getPointOnB();
      Vector3D normalOnB = resultToPack.getNormalOnB();

      double sumOfSquares = EuclidCoreTools.normSquared(xLocalQuery / xRadius, yLocalQuery / yRadius, zLocalQuery / zRadius);

      if (sumOfSquares > 1.0e-10)
      {
         double scaleFactor = 1.0 / Math.sqrt(sumOfSquares);

         pointOnB.sub(point3D, ellipsoid3DPose.getShapePosition());
         pointOnB.scale(scaleFactor);
         pointOnB.add(ellipsoid3DPose.getShapePosition());

         double xScale = 1.0 / (xRadius * xRadius);
         double yScale = 1.0 / (yRadius * yRadius);
         double zScale = 1.0 / (zRadius * zRadius);

         normalOnB.set(xLocalQuery, yLocalQuery, zLocalQuery);
         normalOnB.scale(xScale, yScale, zScale);
         normalOnB.normalize();
         ellipsoid3DPose.transform(normalOnB);

         double signedDistance = point3D.distance(ellipsoid3DPose.getShapePosition()) * (1.0 - scaleFactor);
         if (signedDistance < 0.0)
         {
            resultToPack.setShapesAreColliding(true);
            resultToPack.setDepth(-signedDistance);
         }
         else
         {
            resultToPack.setShapesAreColliding(false);
            resultToPack.setDistance(signedDistance);
         }
      }
      else
      {
         pointOnB.scaleAdd(zRadius, ellipsoid3DPose.getZAxis(), ellipsoid3DPose.getShapePosition());
         normalOnB.set(ellipsoid3DPose.getZAxis());

         resultToPack.setShapesAreColliding(true);
         resultToPack.setDepth(zRadius - zLocalQuery);
      }

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(normalOnB);
   }

   public static void doPointShape3DRamp3DCollisionTest(PointShape3DReadOnly pointShape3D, Ramp3DReadOnly ramp3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DRamp3DCollisionTest(pointShape3D, ramp3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(ramp3D);
   }

   public static void doSphere3DRamp3DCollisionTest(Sphere3DReadOnly sphere3D, Ramp3DReadOnly ramp3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DRamp3DCollisionTest(sphere3D.getPosition(), ramp3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(ramp3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DRamp3DCollisionTest(Point3DReadOnly query, Ramp3DReadOnly ramp3D, Shape3DCollisionTestResult resultToPack)
   {
      resultToPack.setToNaN();

      Shape3DPoseReadOnly ramp3DPose = ramp3D.getPose();
      Vector3DReadOnly ramp3DSize = ramp3D.getSize();

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

      Point3D pointOnB = resultToPack.getPointOnB();
      Vector3D normalOnB = resultToPack.getNormalOnB();

      if (zLocalQuery < 0.0)
      { // Query is below the ramp
         resultToPack.setShapesAreColliding(false);
         double xClosest = EuclidCoreTools.clamp(xLocalQuery, 0.0, ramp3DSize.getX());
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         pointOnB.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(pointOnB);

         if (xClosest == xLocalQuery && yClosest == yLocalQuery)
         {
            normalOnB.setAndNegate(ramp3DPose.getZAxis());
            resultToPack.setDistance(-zLocalQuery);
         }
         else
         {
            normalOnB.sub(query, pointOnB);
            double distance = normalOnB.length();
            normalOnB.scale(1.0 / distance);
            resultToPack.setDistance(distance);
         }
      }
      else if (xLocalQuery > ramp3DSize.getX()
            || EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, ramp3DSize.getX(), ramp3DSize.getZ(), rampNormalX, rampNormalZ, false))
      { // Query is beyond the ramp
         resultToPack.setShapesAreColliding(false);
         double xClosest = ramp3DSize.getX();
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = EuclidCoreTools.clamp(zLocalQuery, 0.0, ramp3DSize.getZ());

         pointOnB.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(pointOnB);

         if (yClosest == yLocalQuery && zClosest == zLocalQuery)
         {
            normalOnB.set(ramp3DPose.getXAxis());
            resultToPack.setDistance(xLocalQuery - ramp3DSize.getX());
         }
         else
         {
            normalOnB.sub(query, pointOnB);
            double distance = normalOnB.length();
            normalOnB.scale(1.0 / distance);
            resultToPack.setDistance(distance);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampNormalX, rampNormalZ, true))
      { // Query is before ramp and the closest point lies on starting edge
         resultToPack.setShapesAreColliding(false);
         double xClosest = 0.0;
         double yClosest = EuclidCoreTools.clamp(yLocalQuery, -halfWidth, halfWidth);
         double zClosest = 0.0;

         pointOnB.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(pointOnB);
         normalOnB.sub(query, pointOnB);
         double distance = normalOnB.length();
         normalOnB.scale(1.0 / distance);
         resultToPack.setDistance(distance);
      }
      else if (Math.abs(yLocalQuery) > halfWidth)
      { // Query is on either side of the ramp
         resultToPack.setShapesAreColliding(false);
         double yClosest = Math.copySign(halfWidth, yLocalQuery);

         if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, false))
         { // Query is below the slope
            double xClosest = xLocalQuery;
            double zClosest = zLocalQuery;

            pointOnB.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(pointOnB);

            if (yLocalQuery >= 0.0)
               normalOnB.set(ramp3DPose.getYAxis());
            else
               normalOnB.setAndNegate(ramp3DPose.getYAxis());

            resultToPack.setDistance(Math.abs(yLocalQuery) - halfWidth);
         }
         else
         { // Query is above the slope
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double zClosest = dot * rampDirectionZ;

            pointOnB.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(pointOnB);

            normalOnB.sub(query, pointOnB);
            double distance = normalOnB.length();
            normalOnB.scale(1.0 / distance);
            resultToPack.setDistance(distance);
         }
      }
      else if (EuclidGeometryTools.isPoint2DOnSideOfLine2D(xLocalQuery, zLocalQuery, 0.0, 0.0, rampDirectionX, rampDirectionZ, true))
      { // Query is directly above the slope part
         resultToPack.setShapesAreColliding(false);
         double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
         double xClosest = dot * rampDirectionX;
         double yClosest = yLocalQuery;
         double zClosest = dot * rampDirectionZ;

         pointOnB.set(xClosest, yClosest, zClosest);
         ramp3DPose.transform(pointOnB);

         normalOnB.set(rampNormalX, 0.0, rampNormalZ);
         ramp3DPose.transform(normalOnB);
         resultToPack.setDistance(rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ);
      }
      else
      { // Query is inside the ramp
         resultToPack.setShapesAreColliding(true);
         double distanceToRightFace = -(-halfWidth - yLocalQuery);
         double distanceToLeftFace = halfWidth - yLocalQuery;
         double distanceToRearFace = ramp3DSize.getX() - xLocalQuery;
         double distanceToBottomFace = zLocalQuery;
         double distanceToSlopeFace = -(rampDirectionX * zLocalQuery - xLocalQuery * rampDirectionZ);

         if (isFirstValueMinimum(distanceToRightFace, distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the right face
            pointOnB.set(xLocalQuery, -halfWidth, zLocalQuery);
            ramp3DPose.transform(pointOnB);

            normalOnB.setAndNegate(ramp3DPose.getYAxis());
            resultToPack.setDepth(distanceToRightFace);
         }
         else if (isFirstValueMinimum(distanceToLeftFace, distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the left face
            pointOnB.set(xLocalQuery, halfWidth, zLocalQuery);
            ramp3DPose.transform(pointOnB);

            normalOnB.set(ramp3DPose.getYAxis());
            resultToPack.setDepth(distanceToLeftFace);
         }
         else if (isFirstValueMinimum(distanceToRearFace, distanceToBottomFace, distanceToSlopeFace))
         { // Query is closer to the rear face
            pointOnB.set(ramp3DSize.getX(), yLocalQuery, zLocalQuery);
            ramp3DPose.transform(pointOnB);
            normalOnB.set(ramp3DPose.getXAxis());
            resultToPack.setDepth(distanceToRearFace);
         }
         else if (distanceToBottomFace <= distanceToSlopeFace)
         { // Query is closer to the bottom face
            pointOnB.set(xLocalQuery, yLocalQuery, 0.0);
            ramp3DPose.transform(pointOnB);
            normalOnB.setAndNegate(ramp3DPose.getZAxis());
            resultToPack.setDepth(distanceToBottomFace);
         }
         else
         { // Query is closer to the slope face
            double dot = xLocalQuery * rampDirectionX + zLocalQuery * rampDirectionZ;
            double xClosest = dot * rampDirectionX;
            double yClosest = yLocalQuery;
            double zClosest = dot * rampDirectionZ;

            pointOnB.set(xClosest, yClosest, zClosest);
            ramp3DPose.transform(pointOnB);
            normalOnB.set(rampNormalX, 0.0, rampNormalZ);
            ramp3DPose.transform(normalOnB);
            resultToPack.setDepth(distanceToSlopeFace);
         }
      }

      resultToPack.getPointOnA().set(query);
      resultToPack.getNormalOnA().setAndNegate(normalOnB);
   }

   public static void doPointShape3DSphere3DCollisionTest(PointShape3DReadOnly pointShape3D, Sphere3DReadOnly sphere3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DSphere3DCollisionTest(pointShape3D, sphere3D.getPosition(), sphere3D.getRadius(), resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(sphere3D);
   }

   public static void doSphere3DSphere3DCollisionTest(Sphere3DReadOnly sphere3DA, Sphere3DReadOnly sphere3DB, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DSphere3DCollisionTest(sphere3DA.getPosition(), sphere3DB.getPosition(), sphere3DB.getRadius(), resultToPack);
      resultToPack.setShapeA(sphere3DA);
      resultToPack.setShapeB(sphere3DB);

      resultToPack.getPointOnA().scaleAdd(sphere3DA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3DA.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DSphere3DCollisionTest(Point3DReadOnly point3D, Point3DReadOnly sphere3DPosition, double sphere3DRadius,
                                                      Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DSphere3DCollisionTest(point3D, sphere3DRadius, sphere3DPosition.getX(), sphere3DPosition.getY(), sphere3DPosition.getZ(), resultToPack);
   }

   private static void doPoint3DSphere3DCollisionTest(Point3DReadOnly point3D, double sphere3DRadius, double sphere3DPositionX, double sphere3DPositionY,
                                                      double sphere3DPositionZ, Shape3DCollisionTestResult resultToPack)
   {
      resultToPack.setToNaN();

      double distanceFromCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(sphere3DPositionX, sphere3DPositionY, sphere3DPositionZ, point3D);

      Point3D pointOnB = resultToPack.getPointOnB();
      Vector3D normalOnB = resultToPack.getNormalOnB();

      if (distanceFromCenter > SPHERE_SMALLEST_DISTANCE_TO_ORIGIN)
      {
         normalOnB.set(sphere3DPositionX, sphere3DPositionY, sphere3DPositionZ);
         normalOnB.sub(point3D, normalOnB);
         normalOnB.scale(1.0 / distanceFromCenter);
         pointOnB.set(normalOnB);
         pointOnB.scale(sphere3DRadius);
      }
      else
      {
         pointOnB.set(0.0, 0.0, sphere3DRadius);
         normalOnB.set(0.0, 0.0, 1.0);
      }
      pointOnB.add(sphere3DPositionX, sphere3DPositionY, sphere3DPositionZ);

      double signedDistance = distanceFromCenter - sphere3DRadius;

      if (signedDistance < 0.0)
      {
         resultToPack.setShapesAreColliding(true);
         resultToPack.setDepth(-signedDistance);
      }
      else
      {
         resultToPack.setShapesAreColliding(false);
         resultToPack.setDistance(signedDistance);
      }

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
   }

   public static void doPointShape3DTorus3DCollisionTest(PointShape3DReadOnly pointShape3D, Torus3DReadOnly torus3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DTorus3DCollisionTest(pointShape3D, torus3D, resultToPack);
      resultToPack.setShapeA(pointShape3D);
      resultToPack.setShapeB(torus3D);
   }

   public static void doSphere3DTorus3DCollisionTest(Sphere3DReadOnly sphere3D, Torus3DReadOnly torus3D, Shape3DCollisionTestResult resultToPack)
   {
      doPoint3DTorus3DCollisionTest(sphere3D.getPosition(), torus3D, resultToPack);
      resultToPack.setShapeA(sphere3D);
      resultToPack.setShapeB(torus3D);

      resultToPack.getPointOnA().scaleAdd(sphere3D.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      if (resultToPack.areShapesColliding())
         return;

      double distance = resultToPack.getDistance() - sphere3D.getRadius();

      if (distance > 0.0)
         return;

      resultToPack.setShapesAreColliding(true);
      resultToPack.setDepth(distance);
      resultToPack.setDistance(Double.NaN);
   }

   private static void doPoint3DTorus3DCollisionTest(Point3DReadOnly point3D, Torus3DReadOnly torus3D, Shape3DCollisionTestResult resultToPack)
   {
      double torus3DRadius = torus3D.getRadius();
      double torus3DTubeRadius = torus3D.getTubeRadius();
      Vector3DReadOnly torus3DAxis = torus3D.getAxis();
      Point3DReadOnly torus3DPosition = torus3D.getPosition();

      double x = point3D.getX() - torus3DPosition.getX();
      double y = point3D.getY() - torus3DPosition.getY();
      double z = point3D.getZ() - torus3DPosition.getZ();
      double percentageOnAxis = TupleTools.dot(x, y, z, torus3DAxis);

      double xInPlane = x - percentageOnAxis * torus3DAxis.getX();
      double yInPlane = y - percentageOnAxis * torus3DAxis.getY();
      double zInPlane = z - percentageOnAxis * torus3DAxis.getZ();

      double distanceSquaredFromAxis = EuclidCoreTools.normSquared(xInPlane, yInPlane, zInPlane);

      Point3D pointOnB = resultToPack.getPointOnB();
      Vector3D normalOnB = resultToPack.getNormalOnB();

      if (distanceSquaredFromAxis < TORUS_SMALLEST_DISTANCE_TO_AXIS)
      {
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

         pointOnB.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         pointOnB.scale(resultDistanceFromAxis / norm);
         pointOnB.scaleAdd(resultPositionOnAxis, torus3DAxis, pointOnB);
         pointOnB.add(torus3DPosition);

         normalOnB.set(xOrthogonalToAxis, yOrthogonalToAxis, zOrthogonalToAxis);
         normalOnB.scale(-torus3DRadius / norm);
         normalOnB.scaleAdd(percentageOnAxis, torus3DAxis, normalOnB);
         normalOnB.scale(1.0 / distanceFromTubeCenter);

         double signedDistance = distanceFromTubeCenter - torus3DTubeRadius;

         if (signedDistance < 0.0)
         {
            resultToPack.setShapesAreColliding(true);
            resultToPack.setDepth(-signedDistance);
         }
         else
         {
            resultToPack.setShapesAreColliding(false);
            resultToPack.setDistance(signedDistance);
         }
      }
      else
      {
         double distanceFromAxis = Math.sqrt(distanceSquaredFromAxis);

         double scale = torus3DRadius / distanceFromAxis;

         double xTubeCenter = xInPlane * scale + torus3DPosition.getX();
         double yTubeCenter = yInPlane * scale + torus3DPosition.getY();
         double zTubeCenter = zInPlane * scale + torus3DPosition.getZ();

         double distanceFromTubeCenter = EuclidGeometryTools.distanceBetweenPoint3Ds(xTubeCenter, yTubeCenter, zTubeCenter, point3D);

         normalOnB.set(point3D);
         normalOnB.sub(xTubeCenter, yTubeCenter, zTubeCenter);
         normalOnB.scale(1.0 / distanceFromTubeCenter);

         pointOnB.setAndScale(torus3DTubeRadius, normalOnB);
         pointOnB.add(xTubeCenter, yTubeCenter, zTubeCenter);

         double signedDistance = distanceFromTubeCenter - torus3DTubeRadius;

         if (signedDistance < 0.0)
         {
            resultToPack.setShapesAreColliding(true);
            resultToPack.setDepth(-signedDistance);
         }
         else
         {
            resultToPack.setShapesAreColliding(false);
            resultToPack.setDistance(signedDistance);
         }
      }

      resultToPack.getPointOnA().set(point3D);
      resultToPack.getNormalOnA().setAndNegate(resultToPack.getNormalOnB());
   }
}
