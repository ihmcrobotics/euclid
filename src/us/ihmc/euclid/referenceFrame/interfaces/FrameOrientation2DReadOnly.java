package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public interface FrameOrientation2DReadOnly extends Orientation2DReadOnly, ReferenceFrameHolder
{   
   /**
    * Computes and returns the difference between {@code this} and {@code other}:<br>
    * {@code distance = this.yaw - other.yaw}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the difference between {@code this} and {@code other} contained in [-<i>pi</i>,
    *         <i>pi</pi>].
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double difference(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      
      return Orientation2DReadOnly.super.difference(other);
   }

   /**
    * Computes the distance between {@code this} and {@code other} as the absolute difference in
    * angle:<br>
    * {@code distance = Math.abs(this.yaw - other.yaw)}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the distance between {@code this} and {@code other} contained in [0, <i>pi</pi>].
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distance(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      
      return Orientation2DReadOnly.super.distance(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default boolean epsilonEquals(FrameOrientation2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      
      return Orientation2DReadOnly.super.epsilonEquals(other, epsilon);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default boolean geometricallyEquals(FrameOrientation2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      
      return Orientation2DReadOnly.super.geometricallyEquals(other, epsilon);
   }
   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by
    * this.
    *
    * <pre>
    * tupleToTransform = / cos(yaw) -sin(yaw) \ * tupleToTransform
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple2D tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      
      Orientation2DReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    * tupleTransformed = / cos(yaw) -sin(yaw) \ * tupleOriginal
    *                    \ sin(yaw)  cos(yaw) /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given {@code tupleToTransform} by the rotation about the z-axis described by
    * this.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleToTransform = | sin(yaw)  cos(yaw) 0 | * tupleToTransform
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple3D tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      
      Orientation2DReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given {@code tupleOriginal} by the rotation about the z-axis described by this
    * and stores the result in {@code tupleTransformed}.
    *
    * <pre>
    *                    / cos(yaw) -sin(yaw) 0 \
    * tupleTransformed = | sin(yaw)  cos(yaw) 0 | * tupleOriginal
    *                    \    0         0     1 /
    * </pre>
    *
    * @param tupleOriginal the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation
    * about the z-axis described by this.
    *
    * <pre>
    * tupleToTransform = / cos(-yaw) -sin(-yaw) \ * tupleToTransform
    *                    \ sin(-yaw)  cos(-yaw) /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple2D tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      
      Orientation2DReadOnly.super.inverseTransform(tupleToTransform);
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
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);

      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());

      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
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
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given {@code tupleToTransform} by the rotation
    * about the z-axis described by this.
    *
    * <pre>
    *                    / cos(-yaw) -sin(-yaw) 0 \
    * tupleToTransform = | sin(-yaw)  cos(-yaw) 0 | * tupleToTransform
    *                    \     0          0     1 /
    * </pre>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if {@code tupleToTransform} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple3D tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      
      Orientation2DReadOnly.super.inverseTransform(tupleToTransform);
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
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
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
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if {@code tupleOriginal} is not expressed in the same frame as
    *            {@code this}.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      
      tupleTransformed.setToZero(getReferenceFrame());
      
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }
}
