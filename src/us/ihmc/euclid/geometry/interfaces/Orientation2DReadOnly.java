package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read-only interface for a 2D orientation.
 * <p>
 * A 2D orientation is in the XY-plane, i.e. the yaw angle about the z-axis.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface Orientation2DReadOnly
{
   /**
    * Returns the current yaw angle of this orientation 2D.
    *
    * @return the angle value in radians.
    */
   double getYaw();

   /**
    * Tests if this orientation 2D contains {@link Double#NaN}.
    *
    * @return {@code true} if this orientation 2D contains a {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean containsNaN()
   {
      return Double.isNaN(getYaw());
   }

   /**
    * Computes and returns the difference between {@code this} and {@code other}:<br>
    * {@code distance = this.yaw - other.yaw}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the difference between {@code this} and {@code other} contained in [-<i>pi</i>,
    *         <i>pi</pi>].
    */
   default double difference(Orientation2DReadOnly other)
   {
      return EuclidCoreTools.angleDifferenceMinusPiToPi(getYaw(), other.getYaw());
   }

   /**
    * Computes the distance between {@code this} and {@code other} as the absolute difference in
    * angle:<br>
    * {@code distance = Math.abs(this.yaw - other.yaw)}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the distance between {@code this} and {@code other} contained in [0, <i>pi</pi>].
    */
   default double distance(Orientation2DReadOnly other)
   {
      return Math.abs(difference(other));
   }

   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by this.
    *
    * <pre>
    * tupleToTransform = / cos(yaw) -sin(yaw) \ * tupleToTransform
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this and
    * stores the result in {@code tupleTransformed}.
    *
    * <pre>
    * tupleTransformed = / cos(yaw) -sin(yaw) \ * tupleOriginal
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(getYaw(), tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by this.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleToTransform = | sin(yaw)  cos(yaw) 0 | * tupleToTransform
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this and
    * stores the result in {@code tupleTransformed}.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(getYaw(), tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation about
    * the z-axis described by this.
    *
    * <pre>
    * tupleToTransform = / cos(-yaw) -sin(-yaw) \ * tupleToTransform
    *                    \ sin(-yaw)  cos(-yaw) /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleOriginal} by the rotation about
    * the z-axis described by this and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    * tupleTransformed = / cos(-yaw) -sin(-yaw) \ * tupleOriginal
    *                    \ sin(-yaw)  cos(-yaw) /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(-getYaw(), tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation about
    * the z-axis described by this.
    *
    * <pre>
    *                    / cos(-yaw) -sin(-yaw) 0 \
    * tupleToTransform = | sin(-yaw)  cos(-yaw) 0 | * tupleToTransform
    *                    \     0          0     1 /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleOriginal} by the rotation about
    * the z-axis described by this and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    *                    / cos(-yaw) -sin(-yaw) 0 \
    * tupleTransformed = | sin(-yaw)  cos(-yaw) 0 | * tupleOriginal
    *                    \     0          0     1 /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      RotationMatrixTools.applyYawRotation(-getYaw(), tupleOriginal, tupleTransformed);
   }

   /**
    * Tests if the yaw angle of this orientation is equal to an {@code epsilon} to the yaw of
    * {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two orientations are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Orientation2DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getYaw(), other.getYaw(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two orientations are geometrically
    * similar, i.e. the difference in yaw of {@code this} and {@code other} is less than or equal to
    * {@code epsilon}.
    *
    * @param other the orientation to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(Orientation2DReadOnly other, double epsilon)
   {
      return Math.abs(difference(other)) <= epsilon;
   }

   /**
    * Tests if this orientation 2D is exactly equal to {@code other}.
    * <p>
    * Note that this method performs number comparison and not an angle comparison, such that:
    * -<i>pi</i> &ne; <i>pi</i>.
    * </p>
    *
    * @param other the other orientation 2D to compare against this. Not modified.
    * @return {@code true} if the two orientations are exactly equal, {@code false} otherwise.
    */
   default boolean equals(Orientation2DReadOnly other)
   {
      if (other == null)
         return false;
      else
         return getYaw() == other.getYaw();
   }
}
