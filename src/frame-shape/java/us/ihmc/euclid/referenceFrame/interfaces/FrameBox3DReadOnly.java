package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface FrameBox3DReadOnly extends Box3DReadOnly, FrameShape3DReadOnly
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
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxBox3D(this, destinationFrame, boundingBoxToPack);
   }

   default FramePoint3DBasics[] getVertices()
   {
      FramePoint3D[] vertices = new FramePoint3D[8];
      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, vertices[vertexIndex] = new FramePoint3D());
      return vertices;
   }

   default void getVertices(FramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   default void getVertices(FixedFramePoint3DBasics[] verticesToPack)
   {
      if (verticesToPack.length < 8)
         throw new IllegalArgumentException("Array is too small, has to be at least 8 element long, was: " + verticesToPack.length);

      for (int vertexIndex = 0; vertexIndex < 8; vertexIndex++)
         getVertex(vertexIndex, verticesToPack[vertexIndex]);
   }

   default FramePoint3DBasics getVertex(int vertexIndex)
   {
      FramePoint3D vertex = new FramePoint3D();
      getVertex(vertexIndex, vertex);
      return vertex;
   }

   default void getVertex(int vertexIndex, FramePoint3DBasics vertexToPack)
   {
      getVertex(vertexIndex, (Point3DBasics) vertexToPack);
      vertexToPack.setReferenceFrame(getReferenceFrame());
   }

   default void getVertex(int vertexIndex, FixedFramePoint3DBasics vertexToPack)
   {
      checkReferenceFrameMatch(vertexToPack);
      getVertex(vertexIndex, (Point3DBasics) vertexToPack);
   }

   default boolean epsilonEquals(FrameBox3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return Box3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Box3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   default boolean equals(FrameBox3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPose().equals(other.getPose()) && getSize().equals(other.getSize());
   }
}
