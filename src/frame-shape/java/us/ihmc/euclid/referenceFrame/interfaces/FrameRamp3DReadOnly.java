package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;

public interface FrameRamp3DReadOnly extends Ramp3DReadOnly, FrameShape3DReadOnly
{
   @Override
   FrameVector3DReadOnly getSize();

   @Override
   FrameShape3DPoseReadOnly getPose();

   @Override
   default FrameRotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   @Override
   default FramePoint3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      Ramp3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxRamp3D(this, destinationFrame, boundingBoxToPack);
   }

   @Override
   default FrameVector3DBasics getRampSurfaceNormal()
   {
      FrameVector3D surfaceNormal = new FrameVector3D();
      getRampSurfaceNormal(surfaceNormal);
      return surfaceNormal;
   }

   default void getRampSurfaceNormal(FrameVector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.setReferenceFrame(getReferenceFrame());
      Ramp3DReadOnly.super.getRampSurfaceNormal(surfaceNormalToPack);
   }

   default void getRampSurfaceNormal(FixedFrameVector3DBasics surfaceNormalToPack)
   {
      checkReferenceFrameMatch(surfaceNormalToPack);
      Ramp3DReadOnly.super.getRampSurfaceNormal(surfaceNormalToPack);
   }

   @Override
   default FramePoint3DBasics[] getVertices()
   {
      FramePoint3D[] vertices = new FramePoint3D[6];
      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new FramePoint3D());
      return vertices;
   }

   default void getVertices(FramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   default void getVertices(FixedFramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 6)
         throw new IllegalArgumentException("Array is too small, has to be at least 6 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 6; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   @Override
   default FramePoint3DBasics getVertex(int vertexIndex)
   {
      FramePoint3D vertex = new FramePoint3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   default void getVertex(int vertexIndex, FramePoint3DBasics vertexToPack)
   {
      vertexToPack.setReferenceFrame(getReferenceFrame());
      Ramp3DReadOnly.super.getVertex(vertexIndex, vertexToPack);
   }

   default void getVertex(int vertexIndex, FixedFramePoint3DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      Ramp3DReadOnly.super.getVertex(vertexIndex, vertexToPack);
   }

   @Override
   FrameRamp3DReadOnly copy();

   default boolean epsilonEquals(FrameRamp3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Ramp3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameRamp3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Ramp3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameRamp3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }
}
