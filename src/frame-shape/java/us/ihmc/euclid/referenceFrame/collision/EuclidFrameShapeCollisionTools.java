package us.ihmc.euclid.referenceFrame.collision;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;

public class EuclidFrameShapeCollisionTools
{

   public static void evaluatePointShape3DBox3DCollision(FramePointShape3DReadOnly shapeA, FrameBox3DReadOnly shapeB,
                                                         EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DBox3DCollision(shapeA, shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DBox3DCollision(FrameSphere3DReadOnly shapeA, FrameBox3DReadOnly shapeB,
                                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DBox3DCollision(shapeA.getPosition(), shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapesAreColliding(distance < 0.0);
   }

   private static void evaluatePoint3DBox3DCollision(FramePoint3DReadOnly point3D, FrameBox3DReadOnly box3D,
                                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();

      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(point3D);
      ReferenceFrame boxFrame = box3D.getReferenceFrame();
      pointOnA.changeFrame(boxFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      box3D.transformToLocal(pointOnA);

      double distance = EuclidShapeTools.evaluatePoint3DBox3DCollision(pointOnA, box3D.getSize(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(boxFrame);
      normalOnB.setReferenceFrame(boxFrame);
      normalOnA.setReferenceFrame(boxFrame);

      box3D.transformToWorld(pointOnB);
      box3D.transformToWorld(normalOnB);
      pointOnA.setIncludingFrame(boxFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DCapsule3DCollision(FramePointShape3DReadOnly shapeA, FrameCapsule3DReadOnly shapeB,
                                                             EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(shapeA, shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateCapsule3DCapsule3DCollision(FrameCapsule3DReadOnly shapeA, FrameCapsule3DReadOnly shapeB,
                                                          EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      ReferenceFrame frameB = shapeB.getReferenceFrame();
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      pointOnA.setIncludingFrame(shapeA.getTopCenter());
      pointOnA.changeFrame(frameB);
      double topAX = pointOnA.getX();
      double topAY = pointOnA.getY();
      double topAZ = pointOnA.getZ();
      pointOnA.setIncludingFrame(shapeA.getBottomCenter());
      pointOnA.changeFrame(frameB);
      double bottomAX = pointOnA.getX();
      double bottomAY = pointOnA.getY();
      double bottomAZ = pointOnA.getZ();
      FramePoint3DReadOnly topCenterB = shapeB.getTopCenter();
      double topBX = topCenterB.getX();
      double topBY = topCenterB.getY();
      double topBZ = topCenterB.getZ();
      FramePoint3DReadOnly bottomCenterB = shapeB.getBottomCenter();
      double bottomBX = bottomCenterB.getX();
      double bottomBY = bottomCenterB.getY();
      double bottomBZ = bottomCenterB.getZ();

      double distanceBetweenAxes = EuclidGeometryTools.closestPoint3DsBetweenTwoLineSegment3Ds(topAX,
                                                                                               topAY,
                                                                                               topAZ,
                                                                                               bottomAX,
                                                                                               bottomAY,
                                                                                               bottomAZ,
                                                                                               topBX,
                                                                                               topBY,
                                                                                               topBZ,
                                                                                               bottomBX,
                                                                                               bottomBY,
                                                                                               bottomBZ,
                                                                                               pointOnA,
                                                                                               pointOnB);
      pointOnA.setReferenceFrame(frameB);
      pointOnB.setReferenceFrame(frameB);

      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);

      normalOnA.sub(pointOnB, pointOnA);
      normalOnA.scale(1.0 / distanceBetweenAxes);
      normalOnB.setAndNegate(normalOnA);

      pointOnA.scaleAdd(shapeA.getRadius(), normalOnA, pointOnA);
      pointOnB.scaleAdd(shapeB.getRadius(), normalOnB, pointOnB);

      double distance = distanceBetweenAxes - shapeA.getRadius() - shapeB.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);

      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DCapsule3DCollision(FrameSphere3DReadOnly shapeA, FrameCapsule3DReadOnly shapeB,
                                                         EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCapsule3DCollision(shapeA.getPosition(), shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DCapsule3DCollision(FramePoint3DReadOnly point3D, FrameCapsule3DReadOnly capsule3D,
                                                         EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(point3D);
      ReferenceFrame capsuleFrame = capsule3D.getReferenceFrame();
      pointOnA.changeFrame(capsuleFrame);

      double distance = EuclidShapeTools.evaluatePoint3DCapsule3DCollision(pointOnA,
                                                                           capsule3D.getPosition(),
                                                                           capsule3D.getAxis(),
                                                                           capsule3D.getLength(),
                                                                           capsule3D.getRadius(),
                                                                           pointOnB,
                                                                           normalOnB);
      pointOnB.setReferenceFrame(capsuleFrame);
      normalOnA.setReferenceFrame(capsuleFrame);
      normalOnB.setReferenceFrame(capsuleFrame);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DCylinder3DCollision(FramePointShape3DReadOnly shapeA, FrameCylinder3DReadOnly shapeB,
                                                              EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(shapeA, shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DCylinder3DCollision(FrameSphere3DReadOnly shapeA, FrameCylinder3DReadOnly shapeB,
                                                          EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DCylinder3DCollision(shapeA.getPosition(), shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DCylinder3DCollision(FramePoint3DReadOnly point3D, FrameCylinder3DReadOnly cylinder3D,
                                                          EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(point3D);
      ReferenceFrame cylinderFrame = cylinder3D.getReferenceFrame();
      pointOnA.changeFrame(cylinderFrame);

      double distance = EuclidShapeTools.evaluatePoint3DCylinder3DCollision(pointOnA,
                                                                            cylinder3D.getPosition(),
                                                                            cylinder3D.getAxis(),
                                                                            cylinder3D.getLength(),
                                                                            cylinder3D.getRadius(),
                                                                            pointOnB,
                                                                            normalOnB);
      pointOnB.setReferenceFrame(cylinderFrame);
      normalOnA.setReferenceFrame(cylinderFrame);
      normalOnB.setReferenceFrame(cylinderFrame);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DEllipsoid3DCollision(FramePointShape3DReadOnly shapeA, FrameEllipsoid3DReadOnly shapeB,
                                                               EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(shapeA, shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DEllipsoid3DCollision(FrameSphere3DReadOnly shapeA, FrameEllipsoid3DReadOnly shapeB,
                                                           EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DEllipsoid3DCollision(shapeA.getPosition(), shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DEllipsoid3DCollision(FramePoint3DReadOnly point3D, FrameEllipsoid3DReadOnly ellipsoid3D,
                                                           EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(point3D);
      ReferenceFrame ellipsoidFrame = ellipsoid3D.getReferenceFrame();
      pointOnA.changeFrame(ellipsoidFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      ellipsoid3D.transformToLocal(pointOnA);

      double distance = EuclidShapeTools.evaluatePoint3DEllipsoid3DCollision(pointOnA, ellipsoid3D.getRadii(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(ellipsoidFrame);
      normalOnA.setReferenceFrame(ellipsoidFrame);
      normalOnB.setReferenceFrame(ellipsoidFrame);
      ellipsoid3D.transformToWorld(pointOnB);
      ellipsoid3D.transformToWorld(normalOnB);

      pointOnA.setIncludingFrame(ellipsoidFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DPointShape3DCollision(FramePointShape3DReadOnly shapeA, FramePointShape3DReadOnly shapeB,
                                                                EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(shapeA);
      ReferenceFrame frameB = shapeB.getReferenceFrame();
      pointOnA.changeFrame(frameB);
      pointOnB.setIncludingFrame(frameB, shapeB);
      double distance = pointOnA.distance(shapeB);
      normalOnA.setReferenceFrame(frameB);
      normalOnA.sub(pointOnB, pointOnA);
      normalOnB.setReferenceFrame(frameB);
      normalOnB.setAndNegate(normalOnA);

      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      resultToPack.setShapesAreColliding(false);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DRamp3DCollision(FramePointShape3DReadOnly shapeA, FrameRamp3DReadOnly shapeB,
                                                          EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DRamp3DCollision(shapeA, shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DRamp3DCollision(FrameSphere3DReadOnly shapeA, FrameRamp3DReadOnly shapeB,
                                                      EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      evaluatePoint3DRamp3DCollision(shapeA.getPosition(), shapeB, resultToPack);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);

      resultToPack.getPointOnA().scaleAdd(shapeA.getRadius(), resultToPack.getNormalOnA(), resultToPack.getPointOnA());

      double distance = resultToPack.getSignedDistance() - shapeA.getRadius();
      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   private static void evaluatePoint3DRamp3DCollision(FramePoint3DReadOnly point3D, FrameRamp3DReadOnly ramp3D,
                                                      EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(point3D);
      ReferenceFrame rampFrame = ramp3D.getReferenceFrame();
      pointOnA.changeFrame(rampFrame);

      double pointInBBackupX = pointOnA.getX();
      double pointInBBackupY = pointOnA.getY();
      double pointInBBackupZ = pointOnA.getZ();

      ramp3D.transformToLocal(pointOnA);
      double distance = EuclidShapeTools.evaluatePoint3DRamp3DCollision(pointOnA, ramp3D.getSize(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(rampFrame);
      normalOnA.setReferenceFrame(rampFrame);
      normalOnB.setReferenceFrame(rampFrame);

      ramp3D.transformToWorld(pointOnB);
      ramp3D.transformToWorld(normalOnB);
      pointOnA.setIncludingFrame(rampFrame, pointInBBackupX, pointInBBackupY, pointInBBackupZ);
      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
   }

   public static void evaluatePointShape3DSphere3DCollision(FramePointShape3DReadOnly shapeA, FrameSphere3DReadOnly shapeB,
                                                            EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(shapeA);
      ReferenceFrame frameB = shapeB.getReferenceFrame();
      pointOnA.changeFrame(frameB);
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(pointOnA, shapeB.getPosition(), shapeB.getRadius(), pointOnB, normalOnB);
      pointOnB.setReferenceFrame(frameB);
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);

      normalOnA.setAndNegate(normalOnB);

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setFrameShapeA(shapeA);
      resultToPack.setFrameShapeB(shapeB);
   }

   public static void evaluateSphere3DSphere3DCollision(FrameSphere3DReadOnly shapeA, FrameSphere3DReadOnly shapeB,
                                                        EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      FramePoint3DBasics pointOnA = resultToPack.getPointOnA();
      FramePoint3DBasics pointOnB = resultToPack.getPointOnB();
      FrameVector3DBasics normalOnA = resultToPack.getNormalOnA();
      FrameVector3DBasics normalOnB = resultToPack.getNormalOnB();

      pointOnA.setIncludingFrame(shapeA.getPosition());
      ReferenceFrame frameB = shapeB.getReferenceFrame();
      pointOnA.changeFrame(frameB);
      double distance = EuclidShapeTools.evaluatePoint3DSphere3DCollision(pointOnA, shapeB.getPosition(), shapeB.getRadius(), pointOnB, normalOnB);
      distance -= shapeA.getRadius();

      pointOnB.setReferenceFrame(frameB);
      normalOnA.setReferenceFrame(frameB);
      normalOnB.setReferenceFrame(frameB);
      normalOnA.setAndNegate(normalOnB);
      pointOnA.scaleAdd(shapeA.getRadius(), normalOnA, resultToPack.getPointOnA());

      resultToPack.setShapesAreColliding(distance < 0.0);
      resultToPack.setSignedDistance(distance);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
   }
}
