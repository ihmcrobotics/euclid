package us.ihmc.euclid.geometry.exceptions;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;

public class BoundingBoxExceptionTest
{
   private BoundingBox2D test2DBox = new BoundingBox2D();
   private BoundingBox3D test3DBox = new BoundingBox3D();

   @Test
   public void test2DBoundingBoxMessagesCorrect()
   {
      assertEquals("Improper bounding box 2D: " + test2DBox.toString(), new BoundingBoxException(test2DBox).getMessage());
      assertEquals("Improper bounding box 2D: bounding box is null", new BoundingBoxException((BoundingBox2D) null).getMessage());
   }

   @Test
   public void test3DBoundingBoxMessagesCorrect()
   {
      assertEquals("Improper bounding box 3D: " + test3DBox.toString(), new BoundingBoxException(test3DBox).getMessage());
      assertEquals("Improper bounding box 3D: bounding box is null", new BoundingBoxException((BoundingBox3D) null).getMessage());
   }
}
