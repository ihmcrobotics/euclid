package us.ihmc.euclid.shape.collision.gjk;

import static us.ihmc.euclid.shape.collision.gjk.GJKTools.simplexClosestToOrigin;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.shape.collision.EuclidShape3DCollisionResult;
import us.ihmc.euclid.shape.collision.epa.ExpandingPolytopeAlgorithm;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.collision.interfaces.SupportingVertexHolder;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of the Gilbert-Johnson-Keerthi algorithm used for collision detection.
 * <p>
 * The base algorithm is an adaption from the algorithm introduced by Gin van den Bergen in
 * <a href="https://www.taylorfrancis.com/books/9781482297997">Collision Detection in Interactive 3D
 * Environments</a>. The distance sub-algorithm is adapted from the algorithm introduced by Mattia
 * Montanari & Nik Petrinic in <a href="https://dl.acm.org/citation.cfm?id=3083724">Improving the
 * GJK algorithm for faster and more reliable distance queries between convex objects</a>.
 * </p>
 * <p>
 * This collision detector can be used to detect whether two shapes are colliding or not, and when
 * not colliding it provides information about the pair of closest points from each shape. When
 * there is collision, this algorithm does not provide any additional information than the collision
 * is occurring, the {@link ExpandingPolytopeAlgorithm} can then be used to compute the depth of the
 * collision and the collision vector.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class GilbertJohnsonKeerthiCollisionDetector
{
   private static final boolean VERBOSE = false;
   /** The default value for the tolerance used to trigger the terminal condition. */
   public static final double DEFAULT_TERMINAL_CONDITION_EPSILON = 1.0e-16;
   /**
    * When a component of the support direction is exactly zero, this is used to wiggle around zero to
    * force edge-cases to trigger.
    */
   private static final double SUPPORT_DIRECTION_ZERO_COMPONENT = 1.234e-16;
   /**
    * The default value for the tolerance used to switch method for obtaining the next support
    * direction.
    */
   public static final double DEFAULT_EPSILON_SUPPORT_DIRECTION_SWITCH = 1.0e-6;

   /** The tolerance used to trigger the terminal condition. */
   private double epsilon = DEFAULT_TERMINAL_CONDITION_EPSILON;
   /** The tolerance used to switch method for obtaining the next support direction. */
   private double epsilonTriangleNormalSwitch = DEFAULT_EPSILON_SUPPORT_DIRECTION_SWITCH;
   /** The limit to the number of iterations in case the algorithm does not succeed to converge. */
   private int maxIterations = 500;
   /** The number of iterations the last evaluation required. */
   private int numberOfIterations = 0;
   /**
    * The simplex that is the closest to the origin or at the origin resulting from the last collision
    * evaluation.
    */
   private GJKSimplex3D simplex = null;
   /**
    * Flag to indicate whether the initial support direction has been provided by the user or not.
    */
   private boolean isInitialSupportDirectionProvided = false;
   /**
    * The support direction to used for the first iteration. Can be used to reduce the number of
    * iterations.
    */
   private final Vector3D initialSupportDirection = new Vector3D(Axis3D.Y);
   /** The last support direction used in the last evaluation. */
   private final Vector3D supportDirection = new Vector3D();

   /**
    * Creates a new collision detector that can be used right away to evaluate collisions.
    */
   public GilbertJohnsonKeerthiCollisionDetector()
   {
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
   public EuclidShape3DCollisionResult evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
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
   public boolean evaluateCollision(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      boolean areColliding;

      if (!shapeA.isPrimitive() || !shapeB.isPrimitive())
      { // If any of the 2 shapes is not a primitive, doing any copy or transform would probably be expensive. Using the generic approach.
         guessInitialSupportDirection(shapeA, shapeB);
         areColliding = evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, resultToPack);
      }
      else if (shapeA.isDefinedByPose())
      { // Transforming shapeB to be in the local frame of shapeA would save transformations.
         Shape3DPoseReadOnly poseA = shapeA.getPose();
         Shape3DBasics localShapeA = shapeA.copy();
         localShapeA.getPose().setToZero();
         Shape3DBasics localShapeB = shapeB.copy();
         localShapeB.applyInverseTransform(poseA);
         guessInitialSupportDirection(localShapeA, localShapeB);
         areColliding = evaluateCollision((SupportingVertexHolder) localShapeA, (SupportingVertexHolder) localShapeB, resultToPack);
         resultToPack.applyTransform(poseA);
      }
      else if (shapeB.isDefinedByPose())
      { // Transforming shapeA to be in the local frame of shapeB would save transformations.
         Shape3DPoseReadOnly poseB = shapeB.getPose();
         Shape3DBasics localShapeA = shapeA.copy();
         localShapeA.applyInverseTransform(poseB);
         Shape3DBasics localShapeB = shapeB.copy();
         localShapeB.getPose().setToZero();
         guessInitialSupportDirection(localShapeA, localShapeB);
         areColliding = evaluateCollision((SupportingVertexHolder) localShapeA, (SupportingVertexHolder) localShapeB, resultToPack);
         resultToPack.applyTransform(poseB);
      }
      else
      { // None of the 2 shapes is defined with a pose, using the generic algorithm.
         guessInitialSupportDirection(shapeA, shapeB);
         areColliding = evaluateCollision((SupportingVertexHolder) shapeA, (SupportingVertexHolder) shapeB, resultToPack);
      }

      resultToPack.setShapeA(shapeA);
      resultToPack.setShapeB(shapeB);
      return areColliding;
   }

   private void guessInitialSupportDirection(Shape3DReadOnly shapeA, Shape3DReadOnly shapeB)
   {
      if (isInitialSupportDirectionProvided)
         return;

      initialSupportDirection.sub(shapeB.getCentroid(), shapeA.getCentroid());
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
   public EuclidShape3DCollisionResult evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB)
   {
      EuclidShape3DCollisionResult result = new EuclidShape3DCollisionResult();
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
   public boolean evaluateCollision(SupportingVertexHolder shapeA, SupportingVertexHolder shapeB, EuclidShape3DCollisionResultBasics resultToPack)
   {
      GJKSimplex3D previousOutput = new GJKSimplex3D();

      supportDirection.set(initialSupportDirection);
      Point3DReadOnly vertexA = shapeA.getSupportingVertex(supportDirection);
      supportDirection.negate();
      Point3DReadOnly vertexB = shapeB.getSupportingVertex(supportDirection);

      boolean areColliding = false;

      if (vertexA == null || vertexB == null)
      {
         simplex = null;
         areColliding = false;
      }
      else
      {
         double closestPointNormSquared = 1.0;

         for (int i = 0; i < maxIterations; i++)
         {
            numberOfIterations = i;
            GJKVertex3D newVertex = new GJKVertex3D(vertexA, vertexB);

            if (previousOutput.contains(newVertex))
            {
               boolean retry = false;
               supportDirection.negate(); // Undoing the negate from last iteration.

               if (supportDirection.getX() == SUPPORT_DIRECTION_ZERO_COMPONENT)
               {
                  supportDirection.setX(-SUPPORT_DIRECTION_ZERO_COMPONENT);
                  retry = true;
               }
               else if (supportDirection.getY() == SUPPORT_DIRECTION_ZERO_COMPONENT)
               {
                  supportDirection.setY(-SUPPORT_DIRECTION_ZERO_COMPONENT);
                  retry = true;
               }
               else if (supportDirection.getZ() == SUPPORT_DIRECTION_ZERO_COMPONENT)
               {
                  supportDirection.setZ(-SUPPORT_DIRECTION_ZERO_COMPONENT);
                  retry = true;
               }

               if (retry)
               {
                  vertexA = shapeA.getSupportingVertex(supportDirection);
                  supportDirection.negate();
                  vertexB = shapeB.getSupportingVertex(supportDirection);
                  continue;
               }

               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("New vertex belongs to simplex, terminating.");
               break;
            }

            if (Math.abs(closestPointNormSquared - newVertex.dot(supportDirection)) <= epsilon * closestPointNormSquared)
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Progression under tolerance, terminating.");
               break;
            }

            GJKSimplex3D output = simplexClosestToOrigin(previousOutput.getVertices(), newVertex);

            if (output == null)
            { // End of process
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("Closest simplex is null, terminating.");
               break;
            }

            if (output.getDistanceSquaredToOrigin() >= previousOutput.getDistanceSquaredToOrigin())
            {
               simplex = previousOutput;
               areColliding = false;
               if (VERBOSE)
                  System.out.println("No progression, terminating.");
               break;
            }

            closestPointNormSquared = output.getDistanceSquaredToOrigin();

            if (output.getNumberOfVertices() == 4 || closestPointNormSquared <= epsilon * output.getMaxDistanceSquaredToOrigin())
            { // End of process
               simplex = output;
               areColliding = true;
               if (VERBOSE)
                  System.out.println("Collision detected, terminating.");
               break;
            }

            if (closestPointNormSquared < epsilonTriangleNormalSwitch && output.getNumberOfVertices() == 3)
               supportDirection.set(output.getTriangleNormal());
            else
               supportDirection.setAndNegate(output.getClosestPointToOrigin());

            if (Math.abs(supportDirection.getX()) == 0.0)
               supportDirection.setX(SUPPORT_DIRECTION_ZERO_COMPONENT);
            else if (Math.abs(supportDirection.getY()) == 0.0)
               supportDirection.setY(SUPPORT_DIRECTION_ZERO_COMPONENT);
            else if (Math.abs(supportDirection.getZ()) == 0.0)
               supportDirection.setZ(SUPPORT_DIRECTION_ZERO_COMPONENT);

            vertexA = shapeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            vertexB = shapeB.getSupportingVertex(supportDirection);

            previousOutput = output;
         }
      }

      if (areColliding || simplex == null)
      {
         resultToPack.setToNaN();
      }
      else
      {
         resultToPack.setSignedDistance(simplex.getDistanceToOrigin());
         resultToPack.setShapeA(null);
         resultToPack.setShapeB(null);
         simplex.computePointOnA(resultToPack.getPointOnA());
         simplex.computePointOnB(resultToPack.getPointOnB());
         resultToPack.getNormalOnA().setToNaN();
         resultToPack.getNormalOnB().setToNaN();
      }

      resultToPack.setShapesAreColliding(areColliding);

      if (VERBOSE)
         System.out.println("Number of iterations: " + numberOfIterations);

      isInitialSupportDirectionProvided = false;
      initialSupportDirection.set(Axis3D.Y);

      return areColliding;
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
   public void setInitialSupportDirection(Vector3DReadOnly initialSupportDirection)
   {
      isInitialSupportDirectionProvided = true;
      this.initialSupportDirection.set(initialSupportDirection);
   }

   /**
    * Sets the limit to the number of iterations in case the algorithm does not succeed to converge.
    *
    * @param maxIterations the maximum of iterations allowed before terminating.
    */
   public void setMaxIterations(int maxIterations)
   {
      this.maxIterations = maxIterations;
   }

   /**
    * Sets the tolerance used to trigger the termination condition of this algorithm.
    *
    * @param epsilon the terminal condition tolerance to use, default value
    *                {@value #DEFAULT_TERMINAL_CONDITION_EPSILON}.
    */
   public void setTerminalConditionEpsilon(double epsilon)
   {
      this.epsilon = epsilon;
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
    *                value {@value #DEFAULT_EPSILON_SUPPORT_DIRECTION_SWITCH}.
    */
   public void setEpsilonTriangleNormalSwitch(double epsilon)
   {
      epsilonTriangleNormalSwitch = epsilon;
   }

   /**
    * Gets the current value of the tolerance used to trigger the termination condition of this
    * algorithm.
    *
    * @return the current terminal condition tolerance.
    */
   public double getTerminalConditionEpsilon()
   {
      return epsilon;
   }

   /**
    * Gets the current value of the tolerance used to switch method for obtaining the support
    * direction.
    *
    * @return the current tolerance used switch method for obtaining the support direction
    */
   public double getEpsilonTriangleNormalSwitch()
   {
      return epsilonTriangleNormalSwitch;
   }

   /**
    * Gets the number of iterations needed for the last evaluation.
    *
    * @return the number of iterations from the last evaluation.
    */
   public int getNumberOfIterations()
   {
      return numberOfIterations;
   }

   /**
    * Gets the simplex that is the closest to the origin or at the origin resulting from the last
    * collision evaluation.
    *
    * @return the last evaluation resulting simplex.
    */
   public GJKSimplex3D getSimplex()
   {
      return simplex;
   }

   /**
    * Gets the read-only reference to the last support direction used in the last evaluation.
    *
    * @return the last support direction.
    */
   public Vector3DReadOnly getSupportDirection()
   {
      return supportDirection;
   }
}
