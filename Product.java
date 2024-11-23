package com.example.sensemart;

public class Product {
    private String name;
    private String location;
    private String price;
    private int imageResourceId;
    private boolean checked;

    private String imageUrl;
    private String coordinate;


    public Product() {
    }
    public Product(String name, String price, String location, String imageUrl, String coordinate) {
        this.name = name;
        this.price = price;
        this.location = location;
        this.imageUrl = imageUrl;
        this.coordinate = coordinate;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public String getLocation() {
        return location;
    }

    public void setLocation(String location) {
        this.location = location;
    }

    public String getPrice() {
        return price;
    }

    public void setPrice(String price) {
        this.price = price;
    }

    public int getImageResourceId() {
        return imageResourceId;
    }

    public void setImageResourceId(int imageResourceId) {
        this.imageResourceId = imageResourceId;
    }

    public boolean isChecked() {
        return checked;
    }

    public void setChecked(boolean checked) {
        this.checked = checked;
    }

    public String getImageUrl() {
        return imageUrl;
    }

    public void setImageUrl(String imageUrl) {
        this.imageUrl = imageUrl;
    }

    public String getCoordinate(){return coordinate;}
    public void setCoordinate(String coordinate){this.coordinate = coordinate;}
}