package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameTuple2D;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.rotationConversion.RotationVectorConversion;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FrameTuple2D<?, ?> tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform, checkIfTransformInXYPlane);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FrameTuple2D<?, ?> tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FrameTuple3D<?, ?> tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
   default void transform(Tuple3DReadOnly tupleOriginal, FrameTuple3D<?, ?> tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3D<?, ?> tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   // TODO Rename arguments and update doc.

   /**
    * Transforms the given tuple {@code tupleOriginal} by this quaternion and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = quaternion * tupleOriginal * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameQuaternionReadOnly tupleOriginal, QuaternionBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
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
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void transform(FrameQuaternionReadOnly tupleOriginal, FrameQuaternion tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.transform(tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple {@code tupleToTransform} by this quaternion.
    * <p>
    * tupleToTransform = quaternion * tupleToTransform * quaternion<sup>-1</sup>
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void transform(FrameQuaternion tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * quaternion and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this
    * quaternion and stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, FrameTuple3D<?, ?> tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FrameTuple2D<?, ?> tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this quaternion
    *           represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, FrameTuple2D<?, ?> tupleTransformed)
   {
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this quaternion does not represent a transformation in the XY
    *            plane.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple2D<?, ?> tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by
    * this quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modifed.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void inverseTransform(FrameQuaternionReadOnly quaternionOriginal, QuaternionBasics quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleToTransform}.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform * this
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modifed.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleToTransform} do not match.
    */
   default void inverseTransform(FrameTuple3D<?, ?> tupleToTransform)
   {
      checkReferenceFrameMatch(tupleToTransform);
      QuaternionReadOnly.super.inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} and stores the
    * result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal * this
    * </p>
    *
    * @param tupleOriginal the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modifed.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code tupleOriginal} do not match.
    */
   default void inverseTransform(FrameTuple3DReadOnly tupleOriginal, FrameTuple3D<?, ?> tupleTransformed)
   {
      checkReferenceFrameMatch(tupleOriginal);
      tupleTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by
    * this quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modifed.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    */
   default void inverseTransform(QuaternionReadOnly quaternionOriginal, FrameQuaternion quaternionTransformed)
   {
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionOriginal} by
    * this quaternion and stores the result in {@code quaternionTransformed}.
    * <p>
    * quaternionTransformed = this<sup>-1</sup> * quaternionOriginal * this
    * </p>
    *
    * @param quaternionOriginal the quaternion to transform. Not modifed.
    * @param quaternionTransformed the quaternion to store the result. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default void inverseTransform(FrameQuaternionReadOnly quaternionOriginal, FrameQuaternion quaternionTransformed)
   {
      checkReferenceFrameMatch(quaternionOriginal);
      quaternionTransformed.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.inverseTransform(quaternionOriginal, quaternionTransformed);
   }

   /**
    * Performs the inverse of the transform to the given quaternion {@code quaternionToTransform}.
    * <p>
    * quaternionToTransform = this<sup>-1</sup> * quaternionToTransform * this
    * </p>
    *
    * @param quaternionToTransform the quaternion to transform. Not modifed.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionToTransform} do not match.
    */
   default void inverseTransform(FrameQuaternion quaternionToTransform)
   {
      checkReferenceFrameMatch(quaternionToTransform);
      QuaternionReadOnly.super.inverseTransform(quaternionToTransform, quaternionToTransform);
   }

   /**
    * Computes and packs the orientation described by this quaternion as a rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionToTransform} do not match.
    */
   default void get(FrameVector3D rotationVectorToPack)
   {
      rotationVectorToPack.setToZero(getReferenceFrame());
      RotationVectorConversion.convertQuaternionToRotationVector(this, rotationVectorToPack);
   }

   /**
    * Computes and returns the distance from this quaternion to {@code other}.
    *
    * @param other the other quaternion to measure the distance. Not modified.
    * @return the angle representing the distance between the two quaternions. It is contained in
    *         [0, 2<i>pi</i>]
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and
    *            {@code quaternionOriginal} do not match.
    */
   default double distance(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return QuaternionReadOnly.super.distance(other);
   }

   /**
    * Computes and packs the orientation described by this quaternion as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param tupleToTransform the tuple in which the Euler angles are stored. Modified.
    */
   default void getEuler(FrameVector3D eulerAnglesToPack)
   {
      eulerAnglesToPack.setToZero(getReferenceFrame());
      QuaternionReadOnly.super.getEuler(eulerAnglesToPack);
   }
}
