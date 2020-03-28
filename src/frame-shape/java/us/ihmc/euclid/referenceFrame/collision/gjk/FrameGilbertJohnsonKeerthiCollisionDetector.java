package us.ihmc.euclid.referenceFrame.collision.gjk;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GJKSimplex3D;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameGilbertJohnsonKeerthiCollisionDetector
{
   private final SupportingVertexTransformer supportingVertexTransformer = new SupportingVertexTransformer();
   private final GilbertJohnsonKeerthiCollisionDetector gjkCollisionDetector = new GilbertJohnsonKeerthiCollisionDetector();
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private ReferenceFrame detectorFrame;
   private final FrameVector3DReadOnly supportDirection = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(gjkCollisionDetector.getSupportDirection(),
                                                                                                              () -> detectorFrame);
   private final FrameVector3D initialSupportDirection = new FrameVector3D(null, Axis.Y);

   public FrameGilbertJohnsonKeerthiCollisionDetector()
   {
      initialSupportDirection.setToNaN();
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals. In case the two shapes are colliding, this
    * algorithm does not provide any further information. To obtain additional information such as the
    * collision vector for colliding shapes, see {@link ExpandingPolytopeAlgorithm}.
    * </p>
    *
    * @param shapeA the first shape to evaluate. Not modified.
    * @param shapeB the second shape to evaluate. Not modified.
    * @return the collision result.
    */
   public EuclidFrameShape3DCollisionResult evaluateCollision(FrameShape3DReadOnly shapeA, FrameShape3DReadOnly shapeB)
   {
      EuclidFrameShape3DCollisionResult result = new EuclidFrameShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals. In case the two shapes are colliding, this
    * algorithm does not provide any further information. To obtain additional information such as the
    * collision vector for colliding shapes, see {@link ExpandingPolytopeAlgorithm}.
    * </p>
    *
    * @param shapeA       the first shape to evaluate. Not modified.
    * @param shapeB       the second shape to evaluate. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    * @return {@code true} if the shapes are colliding, {@code false} otherwise.
    */
   public boolean evaluateCollision(FrameShape3DReadOnly shapeA, FrameShape3DReadOnly shapeB, EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      boolean areColliding;

      if (shapeA.getReferenceFrame() == shapeB.getReferenceFrame())
      {
         initializeGJK(shapeA.getReferenceFrame());
         areColliding = gjkCollisionDetector.evaluateCollision(shapeA, shapeB, resultToPack);
      }
      else if (!shapeA.isPrimitive())
      {
         if (!shapeB.isPrimitive())
         { // None of the two shapes is a primitive, using the generic approach.
            areColliding = evaluateCollision((SupportingFrameVertexHolder) shapeA, (SupportingFrameVertexHolder) shapeB, resultToPack);
         }
         else
         { // shapeB is a primitive: it can be transformed for cheap.
            if (shapeB.isDefinedByPose())
            {
               /*
                * Slight optimization by concatenating shapeB pose to the transform that'll be used with shapeA
                * reducing the number of transformations by 1 per iteration of the GJK.
                */
               shapeA.getReferenceFrame().getTransformToDesiredFrame(transform, shapeB.getReferenceFrame());
               transform.preMultiplyInvertOther(shapeB.getPose());
               FixedFrameShape3DBasics localShapeB = shapeB.copy();
               localShapeB.getPose().setToZero();
               initializeGJK(shapeB.getReferenceFrame());
               areColliding = evaluateCollision(shapeA, localShapeB, transform, resultToPack);
               resultToPack.applyTransform(shapeB.getPose());
            }
            else
            {
               /*
                * Optimizing by transforming shapeB so it is expressed in shapeA's frame which reduce the number of
                * transformation by 2 per iteration of the GJK.
                */
               shapeB.getReferenceFrame().getTransformToDesiredFrame(transform, shapeA.getReferenceFrame());
               FixedFrameShape3DBasics localShapeB = shapeB.copy();
               localShapeB.applyTransform(transform);
               initializeGJK(shapeA.getReferenceFrame());
               areColliding = gjkCollisionDetector.evaluateCollision(shapeA, localShapeB, resultToPack);
            }
         }
      }
      else if (!shapeB.isPrimitive())
      { // shapeA is a primitive: it can be transformed for cheap.
         if (shapeA.isDefinedByPose())
         {
            /*
             * Slight optimization by concatenating shapeA pose to the transform that'll be used with shapeB
             * reducing the number of transformations by 1 per iteration of the GJK.
             */
            shapeB.getReferenceFrame().getTransformToDesiredFrame(transform, shapeA.getReferenceFrame());
            transform.preMultiplyInvertOther(shapeA.getPose());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.getPose().setToZero();
            initializeGJK(shapeA.getReferenceFrame());
            areColliding = evaluateCollision(shapeB, localShapeA, transform, resultToPack);
            resultToPack.swapShapes();
            resultToPack.applyTransform(shapeA.getPose());
         }
         else
         {
            /*
             * Optimizing by transforming shapeA so it is expressed in shapeB's frame which reduce the number of
             * transformation by 2 per iteration of the GJK.
             */
            shapeA.getReferenceFrame().getTransformToDesiredFrame(transform, shapeB.getReferenceFrame());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.applyTransform(transform);
            initializeGJK(shapeB.getReferenceFrame());
            areColliding = gjkCollisionDetector.evaluateCollision((SupportingVertexHolder) shapeB, (SupportingVertexHolder) localShapeA, resultToPack);
            resultToPack.swapShapes();
         }
      }
      else
      { // Shapes are both primitive
         if (shapeA.isDefinedByPose())
         { // Transforming shapeB to be in the local frame of shapeA would save transformations.
            shapeA.getReferenceFrame().getTransformToDesiredFrame(transform, shapeB.getReferenceFrame());
            transform.multiply(shapeA.getPose());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.getPose().setToZero();
            FixedFrameShape3DBasics localShapeB = shapeB.copy();
            localShapeB.applyInverseTransform(transform);
            initializeGJK(shapeA.getReferenceFrame());
            areColliding = gjkCollisionDetector.evaluateCollision((SupportingVertexHolder) localShapeA, (SupportingVertexHolder) localShapeB, resultToPack);
            resultToPack.applyTransform(shapeA.getPose());
         }
         else if (shapeB.isDefinedByPose())
         { // Transforming shapeA to be in the local frame of shapeB would save transformations.
            shapeB.getReferenceFrame().getTransformToDesiredFrame(transform, shapeA.getReferenceFrame());
            transform.multiply(shapeB.getPose());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.applyInverseTransform(transform);
            FixedFrameShape3DBasics localShapeB = shapeB.copy();
            localShapeB.getPose().setToZero();
            initializeGJK(shapeB.getReferenceFrame());
            areColliding = gjkCollisionDetector.evaluateCollision((SupportingVertexHolder) localShapeA, (SupportingVertexHolder) localShapeB, resultToPack);
            resultToPack.applyTransform(shapeB.getPose());
         }
         else
         { // Transforming shapeA to be in the frame of shapeB would save transformations.
            shapeB.getReferenceFrame().getTransformToDesiredFrame(transform, shapeA.getReferenceFrame());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.applyInverseTransform(transform);
            FixedFrameShape3DBasics localShapeB = shapeB.copy();
            initializeGJK(shapeB.getReferenceFrame());
            areColliding = gjkCollisionDetector.evaluateCollision((SupportingVertexHolder) localShapeA, (SupportingVertexHolder) localShapeB, resultToPack);
         }
      }

      resultToPack.getPointOnA().setReferenceFrame(detectorFrame);
      resultToPack.getPointOnB().setReferenceFrame(detectorFrame);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      return areColliding;
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals. In case the two shapes are colliding, this
    * algorithm does not provide any further information. To obtain additional information such as the
    * collision vector for colliding shapes, see {@link ExpandingPolytopeAlgorithm}.
    * </p>
    *
    * @param shapeA the first shape to evaluate. Not modified.
    * @param shapeB the second shape to evaluate. Not modified.
    * @return the collision result.
    */

   public EuclidFrameShape3DCollisionResult evaluateCollision(SupportingFrameVertexHolder shapeA, SupportingFrameVertexHolder shapeB)
   {
      EuclidFrameShape3DCollisionResult result = new EuclidFrameShape3DCollisionResult();
      evaluateCollision(shapeA, shapeB, result);
      return result;
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals. In case the two shapes are colliding, this
    * algorithm does not provide any further information. To obtain additional information such as the
    * collision vector for colliding shapes, see {@link ExpandingPolytopeAlgorithm}.
    * </p>
    *
    * @param shapeA       the first shape to evaluate. Not modified.
    * @param shapeB       the second shape to evaluate. Not modified.
    * @param resultToPack the object in which the collision result is stored. Modified.
    * @return {@code true} if the shapes are colliding, {@code false} otherwise.
    */
   public boolean evaluateCollision(SupportingFrameVertexHolder shapeA, SupportingFrameVertexHolder shapeB,
                                    EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      shapeA.getReferenceFrame().getTransformToDesiredFrame(transform, shapeB.getReferenceFrame());
      initializeGJK(shapeB.getReferenceFrame());
      boolean areColliding = evaluateCollision(shapeA, shapeB, transform, resultToPack);
      resultToPack.getPointOnA().setReferenceFrame(detectorFrame);
      resultToPack.getPointOnB().setReferenceFrame(detectorFrame);
      return areColliding;
   }

   private void initializeGJK(ReferenceFrame detectorFrame)
   {
      this.detectorFrame = detectorFrame;

      if (initialSupportDirection.getReferenceFrame() != null)
      {
         initialSupportDirection.changeFrame(detectorFrame);
         gjkCollisionDetector.setInitialSupportDirection(initialSupportDirection);
         initialSupportDirection.setReferenceFrame(null);
      }
   }

   private boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, RigidBodyTransformReadOnly transformFromAToB,
                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      supportingVertexTransformer.initialize(shapeA, transformFromAToB);
      return gjkCollisionDetector.evaluateCollision(supportingVertexTransformer, shapeB, resultToPack);
   }

   private static class SupportingVertexTransformer implements SupportingVertexHolder
   {
      private final Vector3D localSupportDirection = new Vector3D();
      private SupportingVertexHolder original;
      private RigidBodyTransformReadOnly transform;

      public void initialize(SupportingVertexHolder original, RigidBodyTransformReadOnly transform)
      {
         this.original = original;
         this.transform = transform;
      }

      @Override
      public boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
      {
         transform.inverseTransform(supportDirection, localSupportDirection);
         boolean success = original.getSupportingVertex(localSupportDirection, supportingVertexToPack);
         if (success)
            transform.transform(supportingVertexToPack);
         return success;
      }
   }

   /**
    * Sets the support direction to use for the first iteration of future evaluations.
    * <p>
    * An appropriate initial support direction can help reducing the number of iterations needed for an
    * evaluation.
    * </p>
    *
    * @param initialSupportDirection the first support direction to use for future collision
    *                                evaluations. Not modified.
    */
   public void setInitialSupportDirection(FrameVector3DReadOnly initialSupportDirection)
   {
      this.initialSupportDirection.setIncludingFrame(initialSupportDirection);
   }

   /**
    * Sets the limit to the number of iterations in case the algorithm does not succeed to converge.
    *
    * @param maxIterations the maximum of iterations allowed before terminating.
    */
   public void setMaxIterations(int maxIterations)
   {
      gjkCollisionDetector.setMaxIterations(maxIterations);
   }

   /**
    * Sets the tolerance used to trigger the termination condition of this algorithm.
    *
    * @param epsilon the terminal condition tolerance to use, default value
    *                {@value GilbertJohnsonKeerthiCollisionDetector#DEFAULT_TERMINAL_CONDITION_EPSILON}.
    */
   public void setTerminalConditionEpsilon(double epsilon)
   {
      gjkCollisionDetector.setTerminalConditionEpsilon(epsilon);
   }

   /**
    * Sets the tolerance used to switch method for obtaining the support direction.
    * <p>
    * In the original algorithm, the support direction for the next iteration is obtained from the
    * coordinates of the closest point on the simplex of the current iteration. When approaching the
    * origin, this method tends to fail and it becomes more reliable to use the closest face normal in
    * case the simplex is a triangle face. This tolerance represents the square of the distance from
    * the origin to the face.
    * </p>
    *
    * @param epsilon the tolerance used switch method for obtaining the support direction, default
    *                value
    *                {@value GilbertJohnsonKeerthiCollisionDetector#DEFAULT_EPSILON_SUPPORT_DIRECTION_SWITCH}.
    */
   public void setEpsilonTriangleNormalSwitch(double epsilon)
   {
      gjkCollisionDetector.setEpsilonTriangleNormalSwitch(epsilon);
   }

   /**
    * Gets the current value of the tolerance used to trigger the termination condition of this
    * algorithm.
    *
    * @return the current terminal condition tolerance.
    */
   public double getTerminalConditionEpsilon()
   {
      return gjkCollisionDetector.getTerminalConditionEpsilon();
   }

   /**
    * Gets the current value of the tolerance used to switch method for obtaining the support
    * direction.
    *
    * @return the current tolerance used switch method for obtaining the support direction
    */
   public double getEpsilonTriangleNormalSwitch()
   {
      return gjkCollisionDetector.getEpsilonTriangleNormalSwitch();
   }

   /**
    * Gets the number of iterations needed for the last evaluation.
    *
    * @return the number of iterations from the last evaluation.
    */
   public int getNumberOfIterations()
   {
      return gjkCollisionDetector.getNumberOfIterations();
   }

   /**
    * Gets the simplex that is the closest to the origin or at the origin resulting from the last
    * collision evaluation.
    *
    * @return the last evaluation resulting simplex.
    */
   public GJKSimplex3D getSimplex()
   {
      return gjkCollisionDetector.getSimplex();
   }

   /**
    * Gets the read-only reference to the last support direction used in the last evaluation.
    *
    * @return the last support direction.
    */
   public FrameVector3DReadOnly getSupportDirection()
   {
      return supportDirection;
   }

   public ReferenceFrame getDetectorFrame()
   {
      return detectorFrame;
   }
}
