package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface FrameQuaternionReadOnly extends FrameTuple4DReadOnly, QuaternionReadOnly
{
   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix
    *            does not represent a transformation in the XY plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.changeFrame(getReferenceFrame());
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleToTransform}
    *            do not match.
    */
   default void transform(FrameTuple2D tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionTools.transform(this, tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleToTransform}
    *            do not match.
    */
   default void transform(FrameTuple2D tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleToTransform}
    *            do not match.
    */
   default void transform(FrameTuple3D tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    *  Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple3D tupleOriginal, FrameTuple3D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      tupleTransformed.changeFrame(getReferenceFrame());
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3D tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.changeFrame(getReferenceFrame());
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameQuaternionReadOnly tupleOriginal, QuaternionBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void transform(QuaternionReadOnly tupleOriginal, FrameQuaternion tupleTransformed)
   {
      tupleTransformed.changeFrame(getReferenceFrame());
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameQuaternionReadOnly tupleOriginal, FrameQuaternion tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.changeFrame(getReferenceFrame());
      QuaternionTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code tupleOriginal}
    *            do not match.
    */
   default void transform(FrameQuaternion tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionTools.transform(this, tupleToTransform, tupleToTransform);
   }
}
