package us.ihmc.euclid.geometry.exceptions;

/**
 * {@link RuntimeException} dedicated to improper use of an empty polygon.
 * 
 * @author Sylvain Bertrand
 *
 */
public class EmptyPolygonException extends RuntimeException
{
   private static final long serialVersionUID = -323833885395952453L;

   /**
    * Creates a new exception.
    * 
    * @param message the detail message.
    */
   public EmptyPolygonException(String message)
   {
      super(message);
   }
}