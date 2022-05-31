package org.firstinspires.ftc.teamcode.RookieTraining;

/*
Mission:
    Create a book object
    Set number of pages
    Say if it is interesting

    Say if your English teacher should assign it
        500+
        not interesting
*/
public class Book {
    //Fields
    public int numPages;
    public boolean isInteresting;
    //Constructor
    public Book(int pages, boolean good_questionMark){
        numPages = pages;
        isInteresting = good_questionMark;
    }
    //Method
    public boolean willTeacherAssign(){
        if(numPages>=500 && !isInteresting){
            return true;
        } else{
            return false;
        }
    }
    public boolean willThomasRead(){
        if(isInteresting && numPages<20){
            return true;
        } else {
            return false;
        }
    }
    public boolean willJamieRead(){
        if(!isInteresting || numPages>800){
            return true;
        }else{
            return false;
        }
    }
}
