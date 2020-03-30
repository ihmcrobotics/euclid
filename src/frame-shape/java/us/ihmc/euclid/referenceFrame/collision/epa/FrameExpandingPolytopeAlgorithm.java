package us.ihmc.euclid.referenceFrame.collision.epa;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.collision.gjk.FrameGilbertJohnsonKeerthiCollisionDetector.SupportingVertexTransformer;
import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.SupportingFrameVertexHolder;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameFactories;
import us.ihmc.euclid.shape.collision.epa.EPAFace3D;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.gjk.GJKSimplex3D;
import us.ihmc.euclid.shape.collision.gjk.GilbertJohnsonKeerthiCollisionDetector;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implementation of the Expanding Polytope algorithm used for collision detection.
 * <p>
 * This class is an extension of {@link ExpandingPolytopeAlgorithm} to handle frame shapes,
 * especially when the shapes are expressed in different reference frames.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class FrameExpandingPolytopeAlgorithm
{
   /** The reference frame in which the last result was evaluated. */
   private ReferenceFrame detectorFrame;
   /** Wrapper around a {@link SupportingVertexHolder} allowing to transform the original object. */
   private final SupportingVertexTransformer supportingVertexTransformer = new SupportingVertexTransformer();
   /** Calculator implementing the actual EPA algorithm. */
   private final ExpandingPolytopeAlgorithm epaAlgorithm = new ExpandingPolytopeAlgorithm();
   /**
    * Wrapper around the GJK last support direction used.
    * <p>
    * The vector's reference frame is linked to {@link #detectorFrame}.
    * </p>
    * <p>
    * The calculator implementing the GJK algorithm is declared as a field of {@link #epaAlgorithm}.
    * </p>
    */
   private final FrameVector3DReadOnly gjkSupportDirection = EuclidFrameFactories.newLinkedFrameVector3DReadOnly(epaAlgorithm.getGJKCollisionDetector()
                                                                                                                             .getSupportDirection(),
                                                                                                                 () -> detectorFrame);
   /**
    * Flag to indicate whether the initial support direction has been provided by the user or not.
    */
   private boolean isInitialSupportDirectionProvided = false;
   /**
    * Duplicate of the GJK initial support direction vector to support frame operations.
    */
   private final FrameVector3D gjkInitialSupportDirection = new FrameVector3D();
   /** Intermediate variable to reduce garbage creation. */
   private final RigidBodyTransform transform = new RigidBodyTransform();
   private final Point3D centroid = new Point3D();

   /**
    * Creates a new collision detector that can be used right away to evaluate collisions.
    */
   public FrameExpandingPolytopeAlgorithm()
   {
      gjkInitialSupportDirection.setToNaN();
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals.
    * </p>
    * <p>
    * This algorithm handles the case when the two shapes are expressed in difference reference frames.
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
    * This algorithm does not evaluate the surface normals.
    * </p>
    * <p>
    * This algorithm handles the case when the two shapes are expressed in difference reference frames.
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
         areColliding = epaAlgorithm.evaluateCollision(shapeA, shapeB, resultToPack);
      }
      else if (!shapeA.isPrimitive())
      {
         if (!shapeB.isPrimitive())
         { // None of the two shapes is a primitive, using the generic approach.
            guessInitialSupportDirection(shapeA, shapeB);
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
               centroid.set(shapeA.getCentroid());
               centroid.applyInverseTransform(transform);
               guessInitialSupportDirection(centroid, localShapeB.getCentroid());
               areColliding = evaluateCollision(shapeB.getReferenceFrame(), shapeA, localShapeB, transform, resultToPack);
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
               guessInitialSupportDirection((Shape3DReadOnly) shapeA, (Shape3DReadOnly) localShapeB);
               areColliding = evaluateCollision(shapeA.getReferenceFrame(), shapeA, localShapeB, resultToPack);
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
            centroid.set(shapeB.getCentroid());
            centroid.applyInverseTransform(transform);
            guessInitialSupportDirection(centroid, localShapeA.getCentroid());
            areColliding = evaluateCollision(shapeA.getReferenceFrame(), shapeB, localShapeA, transform, resultToPack);
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
            guessInitialSupportDirection((Shape3DReadOnly) shapeB, (Shape3DReadOnly) localShapeA);
            areColliding = evaluateCollision(shapeB.getReferenceFrame(), (SupportingVertexHolder) shapeB, (SupportingVertexHolder) localShapeA, resultToPack);
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
            guessInitialSupportDirection((Shape3DReadOnly) localShapeA, (Shape3DReadOnly) localShapeB);
            areColliding = evaluateCollision(shapeA.getReferenceFrame(),
                                             (SupportingVertexHolder) localShapeA,
                                             (SupportingVertexHolder) localShapeB,
                                             resultToPack);
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
            guessInitialSupportDirection((Shape3DReadOnly) localShapeA, (Shape3DReadOnly) localShapeB);
            areColliding = evaluateCollision(shapeB.getReferenceFrame(),
                                             (SupportingVertexHolder) localShapeA,
                                             (SupportingVertexHolder) localShapeB,
                                             resultToPack);
            resultToPack.applyTransform(shapeB.getPose());
         }
         else
         { // Transforming shapeA to be in the frame of shapeB would save transformations.
            shapeB.getReferenceFrame().getTransformToDesiredFrame(transform, shapeA.getReferenceFrame());
            FixedFrameShape3DBasics localShapeA = shapeA.copy();
            localShapeA.applyInverseTransform(transform);
            FixedFrameShape3DBasics localShapeB = shapeB.copy();
            guessInitialSupportDirection(localShapeA, localShapeB);
            areColliding = evaluateCollision(shapeB.getReferenceFrame(),
                                             (SupportingVertexHolder) localShapeA,
                                             (SupportingVertexHolder) localShapeB,
                                             resultToPack);
         }
      }

      resultToPack.getPointOnA().setReferenceFrame(detectorFrame);
      resultToPack.getPointOnB().setReferenceFrame(detectorFrame);
      resultToPack.getNormalOnA().setReferenceFrame(detectorFrame);
      resultToPack.getNormalOnB().setReferenceFrame(detectorFrame);
      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);

      return areColliding;
   }

   private void guessInitialSupportDirection(FrameShape3DReadOnly shapeA, FrameShape3DReadOnly shapeB)
   {
      if (isInitialSupportDirectionProvided)
         return;

      gjkInitialSupportDirection.setReferenceFrame(shapeB.getReferenceFrame());
      gjkInitialSupportDirection.setMatchingFrame(shapeA.getCentroid());
      gjkInitialSupportDirection.negate();
      gjkInitialSupportDirection.add(shapeB.getCentroid());
      isInitialSupportDirectionProvided = true;
   }

   private void guessInitialSupportDirection(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      guessInitialSupportDirection(shapeA.getCentroid(), shapeB.getCentroid());
   }

   private void guessInitialSupportDirection(Point3DReadOnly centroidA, Point3DReadOnly centroidB)
   {
      if (isInitialSupportDirectionProvided)
         return;

      gjkInitialSupportDirection.setReferenceFrame(null);
      gjkInitialSupportDirection.sub(centroidB, centroidA);
      isInitialSupportDirectionProvided = true;
   }

   /**
    * Evaluates the collision state between the two given shapes.
    * <p>
    * This algorithm does not evaluate the surface normals.
    * </p>
    * <p>
    * This algorithm handles the case when the two shapes are expressed in difference reference frames.
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
    * This algorithm does not evaluate the surface normals.
    * </p>
    * <p>
    * This algorithm handles the case when the two shapes are expressed in difference reference frames.
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
      boolean areColliding = evaluateCollision(shapeB.getReferenceFrame(), shapeA, shapeB, transform, resultToPack);
      resultToPack.getPointOnA().setReferenceFrame(detectorFrame);
      resultToPack.getPointOnB().setReferenceFrame(detectorFrame);
      resultToPack.getNormalOnA().setReferenceFrame(detectorFrame);
      resultToPack.getNormalOnB().setReferenceFrame(detectorFrame);
      return areColliding;
   }

   private boolean evaluateCollision(ReferenceFrame detectorFrame, SupportingVertexHolder shapeA, SupportingVertexHolder shapeB,
                                     RigidBodyTransformReadOnly transformFromAToB, EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      supportingVertexTransformer.initialize(shapeA, transformFromAToB);
      return evaluateCollision(detectorFrame, supportingVertexTransformer, shapeB, resultToPack);
   }

   private boolean evaluateCollision(ReferenceFrame detectorFrame, SupportingVertexHolder shapeA, SupportingVertexHolder shapeB,
                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      initializeGJK(detectorFrame);
      return epaAlgorithm.evaluateCollision(shapeA, shapeB, resultToPack);
   }

   private boolean evaluateCollision(ReferenceFrame detectorFrame, Shape3DReadOnly shapeA, Shape3DReadOnly shapeB,
                                     EuclidFrameShape3DCollisionResultBasics resultToPack)
   {
      initializeGJK(detectorFrame);
      return epaAlgorithm.evaluateCollision(shapeA, shapeB, resultToPack);
   }

   private void initializeGJK(ReferenceFrame detectorFrame)
   {
      this.detectorFrame = detectorFrame;

      if (isInitialSupportDirectionProvided)
      {
         if (gjkInitialSupportDirection.getReferenceFrame() != null)
            gjkInitialSupportDirection.changeFrame(detectorFrame);
         epaAlgorithm.getGJKCollisionDetector().setInitialSupportDirection(gjkInitialSupportDirection);
         gjkInitialSupportDirection.setToNaN(null);
         isInitialSupportDirectionProvided = false;
      }
   }

   /**
    * Sets the limit to the number of iterations in case the algorithm does not succeed to converge.
    *
    * @param maxIterations the maximum of iterations allowed before terminating.
    */
   public void setMaxIterations(int maxIterations)
   {
      epaAlgorithm.setMaxIterations(maxIterations);
   }

   /**
    * Sets the tolerance used to trigger the termination condition of this algorithm.
    *
    * @param epsilon the terminal condition tolerance to use, default value
    *                {@value ExpandingPolytopeAlgorithm#DEFAULT_TERMINAL_CONDITION_EPSILON}.
    */
   public void setTerminalConditionEpsilon(double epsilon)
   {
      epaAlgorithm.setTerminalConditionEpsilon(epsilon);
   }

   /**
    * Gets the current value of the tolerance used to trigger the termination condition of this
    * algorithm.
    *
    * @return the current terminal condition tolerance.
    */
   public double getTerminalConditionEpsilon()
   {
      return epaAlgorithm.getTerminalConditionEpsilon();
   }

   /**
    * Gets the number of iterations needed for the last evaluation.
    *
    * @return the number of iterations from the last evaluation.
    */
   public int getNumberOfIterations()
   {
      return epaAlgorithm.getNumberOfIterations();
   }

   /**
    * Gets the face that is the closest to the origin or at the origin resulting from the last
    * collision evaluation.
    *
    * @return the last evaluation resulting face.
    */
   public EPAFace3D getClosestFace()
   {
      return epaAlgorithm.getClosestFace();
   }

   // GJK algorithm parameters
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
   public void setGJKInitialSupportDirection(FrameVector3DReadOnly initialSupportDirection)
   {
      isInitialSupportDirectionProvided = true;
      this.gjkInitialSupportDirection.setIncludingFrame(initialSupportDirection);
   }

   /**
    * Sets the limit to the number of iterations in case the algorithm does not succeed to converge.
    *
    * @param maxIterations the maximum of iterations allowed before terminating.
    */
   public void setGJKMaxIterations(int maxIterations)
   {
      epaAlgorithm.getGJKCollisionDetector().setMaxIterations(maxIterations);
   }

   /**
    * Sets the tolerance used to trigger the termination condition of this algorithm.
    *
    * @param epsilon the terminal condition tolerance to use, default value
    *                {@value GilbertJohnsonKeerthiCollisionDetector#DEFAULT_TERMINAL_CONDITION_EPSILON}.
    */
   public void setGJKTerminalConditionEpsilon(double epsilon)
   {
      epaAlgorithm.getGJKCollisionDetector().setTerminalConditionEpsilon(epsilon);
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
   public void setGJKEpsilonTriangleNormalSwitch(double epsilon)
   {
      epaAlgorithm.getGJKCollisionDetector().setEpsilonTriangleNormalSwitch(epsilon);
   }

   /**
    * Gets the current value of the tolerance used to trigger the termination condition of this
    * algorithm.
    *
    * @return the current terminal condition tolerance.
    */
   public double getGJKTerminalConditionEpsilon()
   {
      return epaAlgorithm.getGJKCollisionDetector().getTerminalConditionEpsilon();
   }

   /**
    * Gets the current value of the tolerance used to switch method for obtaining the support
    * direction.
    *
    * @return the current tolerance used switch method for obtaining the support direction
    */
   public double getEpsilonTriangleNormalSwitch()
   {
      return epaAlgorithm.getGJKCollisionDetector().getEpsilonTriangleNormalSwitch();
   }

   /**
    * Gets the number of iterations needed for the last evaluation.
    *
    * @return the number of iterations from the last evaluation.
    */
   public int getGJKNumberOfIterations()
   {
      return epaAlgorithm.getGJKCollisionDetector().getNumberOfIterations();
   }

   /**
    * Gets the simplex that is the closest to the origin or at the origin resulting from the last
    * collision evaluation.
    *
    * @return the last evaluation resulting simplex.
    */
   public GJKSimplex3D getGJKSimplex()
   {
      return epaAlgorithm.getGJKCollisionDetector().getSimplex();
   }

   /**
    * Gets the read-only reference to the last support direction used in the last evaluation.
    *
    * @return the last support direction.
    */
   public FrameVector3DReadOnly getGJKSupportDirection()
   {
      return gjkSupportDirection;
   }

   public ReferenceFrame getDetectorFrame()
   {
      return detectorFrame;
   }
}
