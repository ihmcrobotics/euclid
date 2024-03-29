package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read-only interface for a 2D orientation that is expressed in a immutable reference frame.
 * <p>
 * A 2D orientation is in the XY-plane, i.e. the yaw angle about the z-axis.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameOrientation2DReadOnly extends Orientation2DReadOnly, EuclidFrameGeometry
{
   /**
    * Computes and returns the difference between {@code this} and {@code other}:<br>
    * {@code distance = this.yaw - other.yaw}
    *
    * @param other the other orientation 2D. Not modified.
    * @return the difference between {@code this} and {@code other} contained in [-<i>pi</i>,
    *         <i>pi</pi>].
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
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
    *                                         {@code this}.
    */
   default double distance(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Orientation2DReadOnly.super.distance(other);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple3DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      Orientation2DReadOnly.super.transform(tupleToTransform);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void transform(FixedFrameTuple2DBasics tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      Orientation2DReadOnly.super.transform(tupleToTransform);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple3DBasics tupleToTransform)
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FixedFrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FixedFrameTuple2DBasics tupleToTransform)
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this},
    *                                         {@code tupleOriginal}, and {@code tupleTransformed} do
    *                                         not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal, tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *                                         {@code tupleTransformed} do not match.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FixedFrameTuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleTransformed);
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
    * @param tupleOriginal    the tuple to be transformed. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2DBasics tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      Orientation2DReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Gets a representative {@code String} of this orientation 2D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * (0.174) - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameOrientation2DString(format, this);
   }
}
