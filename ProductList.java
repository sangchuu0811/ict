package com.example.sensemart;

import android.app.Dialog;
import android.content.Context;
import android.content.Intent;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.os.Bundle;
import android.renderscript.ScriptGroup;
import android.text.TextWatcher;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.view.inputmethod.InputMethodManager;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.Spinner;
import android.widget.TextView;
import android.text.Editable;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import com.algolia.search.saas.Client;
import com.algolia.search.saas.Index;
import com.algolia.search.saas.Query;
import com.algolia.search.saas.CompletionHandler;
import com.algolia.search.saas.AlgoliaException;
import com.bumptech.glide.Glide;
import com.google.firebase.database.DataSnapshot;
import com.google.firebase.database.DatabaseError;
import com.google.firebase.database.DatabaseReference;
import com.google.firebase.database.FirebaseDatabase;
import com.google.firebase.database.ValueEventListener;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.stream.StreamSupport;

public class ProductList extends AppCompatActivity {

    RecyclerView productRecyclerView; // 제품 목록을 표시하기 위한 'RecyclerView'
    Dialog dialog;
    int completedRequests;
    private ArrayList<String> indices; // 여러 제품 인덱스 이름// 을 저장하는 리스트
    private CustomAdapter adapter; // 'RecyclerView'에 데이터를 제공하는 어댑터
    private Client client; // Algolia 검색 client
    String bigCategory;

    @Override
    protected void onCreate(@Nullable Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.product_list_page);
        client = new Client("appid", "apikey"); // Algolia 클라이언트 초기화

        // 리스트에 제품 카테고리 인덱스 추가

        /* 가공식품 */
        indices = new ArrayList<>();
        indices.add("snack_dessert");
        indices.add("healthyfood");
        indices.add("noodles_canned");
        indices.add("mealkits_simplefood");
        indices.add("sidedishes_deli");
        indices.add("bakery_jam");
        indices.add("seasoning_oil");
        indices.add("milk_dailyproducts");
        indices.add("eco_friendly_organic");
        indices.add("drink_liquor");
        indices.add("rice_multi_grain");
        indices.add("coffee_tea");

        /* 신선식품 */
        indices.add("fruit");
        indices.add("vegetable");
        indices.add("meats_eggs");

        /* 화장품 */
        indices.add("hair");
        indices.add("body");
        indices.add("beauty");

        /* 스포츠 */
        indices.add("car");
        indices.add("trip");
        indices.add("workout");

        /* 언더웨어 */
        indices.add("boyfashion");
        indices.add("girlfashion");
        indices.add("unisex");

        /* 생활 */
        indices.add("chidren_toys");
        indices.add("cleaning_housegoods");
        indices.add("dailygoods");
        indices.add("digital_home_appliances");
        indices.add("furniture_interior");
        indices.add("hobby_book");
        indices.add("hygiene_health");
        indices.add("kitchenware");
        indices.add("miscellaneous_luxury_goods");
        indices.add("pet");

        ImageButton back_btn = findViewById(R.id.back_btn);
        back_btn.setOnClickListener(v -> finish());

        ImageButton bakest = findViewById(R.id.basket);
        bakest.setOnClickListener(v -> {
            Intent intent = new Intent(ProductList.this, Bakest.class);
            startActivity(intent);
        });

        productRecyclerView = findViewById(R.id.product_recyclerview);
        productRecyclerView.setLayoutManager(new LinearLayoutManager(this));

        adapter = new CustomAdapter(new ArrayList<>());
        productRecyclerView.setAdapter(adapter);

        dialog = new Dialog(this);
        dialog.requestWindowFeature(Window.FEATURE_NO_TITLE);
        dialog.setContentView(R.layout.dialog);
        Objects.requireNonNull(dialog.getWindow()).setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));

        Intent bigCategoryIntent = getIntent();
        String bigCategory = bigCategoryIntent.getStringExtra("bigcategory");

        Spinner category_spinner = findViewById(R.id.item_spinner);
        ArrayAdapter<CharSequence> spinnerAdapter;

        // bigCategory 값에 따라 스피너의 배열 목록을 다르게 설정
        if (bigCategory != null) {
            switch (bigCategory) {
                case "all":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.allCategory, android.R.layout.simple_spinner_item);
                case "processed_food":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.processedFoodCategory, android.R.layout.simple_spinner_item);
                    break;
                case "fresh_food":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.freshFoodCategory, android.R.layout.simple_spinner_item);
                    break;
                case "cosmetic":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.cosmeticCategory, android.R.layout.simple_spinner_item);
                    break;
                case "sports":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.sportCategory, android.R.layout.simple_spinner_item);
                    break;
                case "underwear":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.underwearCategory, android.R.layout.simple_spinner_item);
                    break;
                case "life":
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.lifeCategory, android.R.layout.simple_spinner_item);
                    break;
                default:
                    // 기본적으로 처리할 내용 (필요시)
                    spinnerAdapter = ArrayAdapter.createFromResource(this,
                            R.array.allCategory, android.R.layout.simple_spinner_item);
                    break;
            }
        } else {
            // bigCategory 값이 없을 때의 기본값 설정
            spinnerAdapter = ArrayAdapter.createFromResource(this,
                    R.array.allCategory, android.R.layout.simple_spinner_item);
        }
        spinnerAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item); // 스피너 레이아웃 지정
        category_spinner.setAdapter(spinnerAdapter); // 어댑터를 스피너에 연결하여 항목들 표시

        Intent categoryIntent = getIntent();
        String selectedCategory = categoryIntent.getStringExtra("category");

        int spinnerPosition = spinnerAdapter.getPosition(selectedCategory); // 선택된 카테고리의 스피너 위치 찾기
        category_spinner.setSelection(spinnerPosition);

        /* 스피너 항목이 선택되었을 때 발생하는 이벤트 처리 */
        category_spinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String selectedCategory = parent.getItemAtPosition(position).toString(); // 선택된 항목의 텍스트 가져오기
                String indexName = getIndexNameForCategory(selectedCategory); // 선택된 카테고리에 해당하는 인덱스 이름을 가져옴
                    searchProducts(indexName); // 인덱스 이름으로 해당 카테고리 상품 검색
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                searchProducts(null);
            }
        });

        /* 상품 검색할 때 발생하는 이벤트 처리 */
        EditText editText = findViewById(R.id.mart_search);

        Intent touchIntent = getIntent();
        boolean EdittextIsTouch = touchIntent.getBooleanExtra("isTouch",false);

        if (EdittextIsTouch) {
            editText.requestFocus();
        }

        editText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) {}

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                searchInMultipleIndices(s.toString());
            }

            @Override
            public void afterTextChanged(Editable s) {}
        });

    }

    private String getIndexNameForCategory(String category) {
        Map<String, String> categoryToIndexMap = new HashMap<>();
//
        /* 가공식품 */
//        categoryToIndexMap.put("전체");// "전체"는 모든 인덱스 검색
        categoryToIndexMap.put("과자/간식", "snack_dessert");
        categoryToIndexMap.put("베이커리/잼", "bakery_jam");
        categoryToIndexMap.put("커피/차", "coffee_tea");
        categoryToIndexMap.put("음료/주류", "drink_liquor");
        categoryToIndexMap.put("우유/유제품", "milk_dailyproducts");
        categoryToIndexMap.put("면류/통조림", "noodles_canned");
        categoryToIndexMap.put("밀키트/간편식", "mealkits_simplefood");
        categoryToIndexMap.put("쌀/잡곡", "rice_multi_grain");
        categoryToIndexMap.put("양념/오일", "seasoning_oil");
        categoryToIndexMap.put("친환경/유기농", "eco_friendly_organic");
        categoryToIndexMap.put("건강식품", "healthyfood");
        categoryToIndexMap.put("반찬/델리", "sidedishes_deli");

        /* 신선식품 */

//        categoryToIndexMap.put("신선식품 전체",);// "전체"는 모든 인덱스 검색
        categoryToIndexMap.put("과일", "fruit");
        categoryToIndexMap.put("채소", "vegetable");
        categoryToIndexMap.put("정육/계란", "meats_eggs");

        /* 화장품 */
        categoryToIndexMap.put("헤어", "hair");
        categoryToIndexMap.put("바디", "body");
        categoryToIndexMap.put("뷰티", "beauty");

        /* 스포츠 */
        categoryToIndexMap.put("스포츠", "workout");
        categoryToIndexMap.put("여행", "trip");
        categoryToIndexMap.put("자동차", "car");

        /* 언더웨어 */
        categoryToIndexMap.put("여성의류", "boyfashion");
        categoryToIndexMap.put("남성의류", "girlfashion");
        categoryToIndexMap.put("남녀공용", "unisex");

        /* 생활 */
        categoryToIndexMap.put("제지/위생/건강", "hygiene_health");
        categoryToIndexMap.put("청소/생활용품", "cleaning_housegoods");
        categoryToIndexMap.put("가구/인테리어", "furniture_interior");
        categoryToIndexMap.put("주방용품", "kitchenware");
        categoryToIndexMap.put("반려동물", "pet");
        categoryToIndexMap.put("생활잡화/공구", "dailygoods");
        categoryToIndexMap.put("유아동/완구", "chideren_toys");
        categoryToIndexMap.put("잡화/명품", "miscellaneous_luxury_goods");
        categoryToIndexMap.put("디지털/가전/렌탈", "digital_home_appliances");
        categoryToIndexMap.put("문구/취미/도서", "hobby_book");

        return categoryToIndexMap.get(category);
    }


/* 스피너 */
    private void searchProducts(String indexName) {
        ArrayList<Product> list = new ArrayList<>(); // 검색 결과를 저장할 Product 객체의 리스트 생성
        // 완료된 요청 수를 추적하기 위한 배열
        // 배열로 만드는 이유는 익명 클래스나 람다 표현식 내부에서 해당 변수를 수정할 수 있도록 위함
        completedRequests = 0;

        List<String> indicesToSearch;

        if (indexName == null) {
            indicesToSearch = indices; // 모든 인덱스 검색
        } else {
            indicesToSearch = new ArrayList<>();
            indicesToSearch.add(indexName); // 특정 인덱스만 검색
        }

        final int totalRequests = indicesToSearch.size();

        for (String currentindexName : indicesToSearch) {
            Index index = client.getIndex(currentindexName); //  검색할 인덱스를 나타내는 Index 객체 생성
            // 인덱스에서 비동기 검색 시작
            // 'new Query("")' 는 인덱스에 있는 모든 데이터를 검색
            // 'setHitsPerPage(100)' 은 검색 결과를 페이지당 최대 100개까지 반환하도록 설정
            index.searchAsync(new Query("").setHitsPerPage(100), new CompletionHandler() {
                @Override
                public void requestCompleted(JSONObject content, AlgoliaException error) {
                    if (error != null) {
                        Log.e("Algolia", "Search error", error);
                        return;
                    }
                    try {
                        // content는 JSONObject 객체이며, 여기서 JSONArray 객체의 'hits'는 검색 결과 항목들을 포함하는 배열이며 이것을 JSONArray 객체에 담음
                        JSONArray hits = content.getJSONArray("hits");
                        for (int i = 0; i < hits.length(); i++) { // 검색 결과 항목들의 갯수만큼 반복문 실행
                            JSONObject json = hits.getJSONObject(i); // 'hits'에서 각 검색 결과 항목을 'JSONObject'로 하나씩 처리
                            try {
                                String name = json.getString("productName");
                                String price = json.getString("productPrice");
                                String location = json.getString("productLocation");
                                String imageUrl = json.optString("url", "");
                                String coordinate = json.getString("coordinate");

                                // 추출한 정보를 사용하여 'Product' 객체를 생성한 후, 이를 리스트에 추가
                                Product pro = new Product(name, price, location, imageUrl,coordinate);
                                list.add(pro);
                                Log.d("Algolia", "Parsed product - Name: " + name + ", Price: " + price + ", Location: " + location + ",coordinate: "+coordinate);
                            } catch (JSONException e) {
                                Log.e("Algolia", "Error parsing product", e);
                            }
                        }
                        completedRequests++; // 완료된 요청 수를 증가

                        if (completedRequests == totalRequests) {
                            // 어댑터 리스트를 갱신하고 초기화
                            adapter.setList(list);
                            adapter.resetting();
                        }
                    } catch (JSONException e) {
                        Log.e("Algolia", "JSON Parsing error", e);
                    }
                }
            });
        }
    }

    /* 검색어*/
    private void searchInMultipleIndices(String query) {
        if (query.isEmpty()) {
            searchProducts(null);
            return;
        }
        //Product.class에 선언된 name,price,location,imageUrl 형식을 사용하는 ArrayList 선언
        ArrayList<Product> list = new ArrayList<>();
        // 요청의 완료 여부를 카운트하지 않는다면, 불완전하게 데이터가 들어올 수 있고,
        // 동일한 코드가 실행될 때마다 다른 결과를 가져올 수 있고, 어느 요청이 완료되지 않았는지 파악하기 어려움
        completedRequests = 0;
        final int totalRequests = indices.size();

        for (String indexName : indices) {
            Index index = client.getIndex(indexName);
            index.searchAsync(new Query(query).setHitsPerPage(100), new CompletionHandler() {
                @Override
                public void requestCompleted(JSONObject content, AlgoliaException error) {
                    if (error != null) {
                        Log.e("Algolia", "Search error", error);
                        return;
                    }
                    try {
                        JSONArray hits = content.getJSONArray("hits");
                        for (int i = 0; i < hits.length(); i++) {
                            JSONObject json = hits.getJSONObject(i);
                            try {
                                String productName = json.getString("productName");
                                String productPrice = json.getString("productPrice");
                                String produceLocation = json.getString("productLocation");
                                String imageUrl = json.optString("url", "");
                                String coordinate = json.getString("coordinate");

                                Product pro = new Product(productName, productPrice, produceLocation, imageUrl,coordinate);
                                list.add(pro);
                                Log.d("Algolia", "Parsed product - Name: " + productName + ", Price: " + productPrice + ", Location: " + produceLocation);
                            } catch (JSONException e) {
                                Log.e("Algolia", "Error parsing product", e);
                            }
                        }

                        completedRequests++;

                        if (completedRequests == totalRequests) {
                            adapter.setList(list);
                            adapter.resetting();
                        }
                    } catch (JSONException e) {
                        Log.e("Algolia", "JSON Parsing error", e);
                    }
                }
            });
        }
    }

    public class CustomAdapter extends RecyclerView.Adapter<CustomAdapter.MyViewHolder> {
        private ArrayList<Product> list;

        public CustomAdapter(ArrayList<Product> list) {
            this.list = list;
        }

        public void setList(ArrayList<Product> list1) {
            list = list1;
        }

        public class MyViewHolder extends RecyclerView.ViewHolder implements View.OnClickListener {
            ImageView productImage;
            TextView productName;
            TextView productLocation;
            TextView productPrice;
            Button bakest_in;

            public MyViewHolder(View itemView) {
                super(itemView);
                productImage = itemView.findViewById(R.id.product_img);
                productName = itemView.findViewById(R.id.product_name);
                productLocation = itemView.findViewById(R.id.product_location);
                productPrice = itemView.findViewById(R.id.product_price);
                bakest_in = itemView.findViewById(R.id.bakest_btn);

                itemView.setOnClickListener(this);
            }

            @Override
            public void onClick(View v) {}
        }

        @NonNull
        @Override
        public MyViewHolder onCreateViewHolder(@NonNull ViewGroup parent, int viewType) {
            View view = LayoutInflater.from(parent.getContext()).inflate(R.layout.product_frame, parent, false);
            return new MyViewHolder(view);
        }

        @Override
        public void onBindViewHolder(@NonNull final MyViewHolder holder, final int position) {
            holder.productName.setText(list.get(position).getName());
            holder.productLocation.setText(list.get(position).getLocation());
            holder.productPrice.setText(list.get(position).getPrice());

            // 이미지 URL을 사용하여 Glide를 통해 이미지를 로드하고 이미지뷰에 표시
            String imageUrl = list.get(position).getImageUrl(); // 이미지 URL 가져옴
            Glide.with(holder.productImage.getContext()) //
                    .load(imageUrl) // 가져온 이미지 URL을 Glide에게 로드
                    .into(holder.productImage); // 이미지 뷰홀더에 추가

            // 가공식품과 신선식품일 때만 영양성분창 띄움
            Intent bigCategoryIntent = getIntent();
            String bigCategory = bigCategoryIntent.getStringExtra("bigcategory");
            if (bigCategory != null && (bigCategory.equals("processed_food") || bigCategory.equals("fresh_food"))) {
                holder.itemView.setOnClickListener(v -> showDialog(position));
            }

            holder.bakest_in.setOnClickListener(v -> {
                String productNameStr = list.get(position).getName();
                String productLocationStr = list.get(position).getLocation();
                String productPriceStr = list.get(position).getPrice();
                String productImageStr = list.get(position).getImageUrl();
                String coordinate = list.get(position).getCoordinate();

                // Firebase에 상품 정보 추가 (Bakest 액티비티로 이동하지 않고)
                DatabaseReference databaseReference = FirebaseDatabase.getInstance().getReference("products");
                addProductToBasket(databaseReference, productNameStr, productLocationStr, productPriceStr, productImageStr, coordinate);
                customToastView("장바구니에 담겼습니다.");
                addWaypoints(coordinate);
            });
        }

        public void addWaypoints(String coordinate) {
            try {
                // 현재 데이터 가져오기
                DatabaseReference waypoints = FirebaseDatabase.getInstance().getReference("admin/marker/waypoints");

                waypoints.addListenerForSingleValueEvent(new ValueEventListener() {
                    @Override
                    public void onDataChange(DataSnapshot dataSnapshot) {
                        List<String> currentWaypoints = new ArrayList<>();

                        // 현재 저장된 데이터를 가져오고, 없으면 빈 리스트 사용
                        if (dataSnapshot.exists()) {
                            String currentData = dataSnapshot.getValue(String.class);
                            if (currentData != null && !currentData.isEmpty()) {
                                // "[]"인 경우 빈 리스트로 처리
                                if (currentData.equals("[]")) {
                                    currentWaypoints = new ArrayList<>();
                                } else {
                                    // 괄호 제거 및 리스트로 변환
                                    currentWaypoints = new ArrayList<>(Arrays.asList(currentData.replace("[", "").replace("]", "").split(", ")));
                                }
                            }
                        }

                        // 새로운 좌표를 기존 데이터에 추가
                        currentWaypoints.add(coordinate);

                        // 전체 데이터를 문자열로 변환하고 괄호로 감싸기
                        StringBuilder finalWaypointsString = new StringBuilder("[");

                        // 리스트의 크기에 따라 요소 추가 및 쉼표 처리
                        for (int i = 0; i < currentWaypoints.size(); i++) {
                            finalWaypointsString.append(currentWaypoints.get(i));
                            // 마지막 요소가 아닐 경우에만 콤마 추가
                            if (i < currentWaypoints.size() - 1) {
                                finalWaypointsString.append(", ");
                            }
                        }

                        // 대괄호 닫기
                        finalWaypointsString.append("]");

                        // Firebase에 설정
                        waypoints.setValue(finalWaypointsString.toString());
                    }

                    @Override
                    public void onCancelled(DatabaseError databaseError) {
                        System.err.println("Failed to read data: " + databaseError.getMessage());
                    }
                });
            } catch (Exception e) {
                System.err.println("Failed to add data to Firebase: " + e.getMessage());
            }
        }






        private void addProductToBasket(DatabaseReference databaseReference, String name, String location, String price, String imageUrl,String coordinate) {
            DatabaseReference productsRef = databaseReference.child(name);
            Product product = new Product(name, price, location, imageUrl,coordinate);
            productsRef.setValue(product);
        }


        public void customToastView(String text){
            LayoutInflater inflater = getLayoutInflater();
            View layout = inflater.inflate(R.layout.toast_custom, (ViewGroup)findViewById(R.id.toast_layout_root));
            TextView textView = layout.findViewById(R.id.textboard);
            textView.setText(text);

            Toast toast = Toast.makeText(getApplicationContext(),text,Toast.LENGTH_SHORT);
            toast.setView(layout);
            toast.show();
        }

        @Override
        public int getItemCount() {
            return list.size();
        }
        public void resetting() {
            notifyDataSetChanged();
        }

        private void showDialog(int position) {
            Button yes_btn = dialog.findViewById(R.id.yes_btn);

            yes_btn.setOnClickListener(v -> dialog.dismiss());
            String dialogName = list.get(position).getName();
            inputDialog(dialogName);

            dialog.show();
        }

        private void inputDialog(String dialogName){
            for (String indexName:indices){
                Index index = client.getIndex(indexName);
                index.searchAsync(new Query(dialogName).setHitsPerPage(10), new CompletionHandler() {
                    @Override
                    public void requestCompleted(@Nullable JSONObject jsonObject, @Nullable AlgoliaException error) {
                        if (error != null) {
                            Log.e("Algolia", "Search error", error);
                            return;
                        }

                        try{
                            JSONArray hits = jsonObject.getJSONArray("hits");
                            for (int i=0; i< hits.length(); i++){
                                JSONObject json = hits.getJSONObject(i);
                                try{
                                    String productName = json.getString("productName");
                                    String productCalory = json.getString("calory");
                                    String productCarbohydrate = json.getString("carbohydrate");
                                    String productFat = json.getString("fat");
                                    String productProtein = json.getString("protein");
                                    String productSodium = json.getString("sodium");
                                    String productVolume = json.getString("volume");

                                    setDialogValues(productName,productCalory,productCarbohydrate,productFat,
                                            productProtein,productSodium,productVolume);
                                }
                                catch (JSONException e) {
                                    Log.e("Algolia", "Error parsing product", e);
                                }
                            }
                        } catch (JSONException ex) {
                            throw new RuntimeException(ex);
                        }
                    }
                });
            }
        }

        private void setDialogValues(String productName, String productCalory,String productCarbohydrate, String productFat, String productProtein, String productSodium, String productVolume){
            TextView infoProductName = dialog.findViewById(R.id.infoproductName);
            TextView capacityValue = dialog.findViewById(R.id.capacity_value);
            TextView calorieValue = dialog.findViewById(R.id.calorie_value);
            TextView carbohydrateValue = dialog.findViewById(R.id.carbohydrate_value);
            TextView proteinValue = dialog.findViewById(R.id.protein_value);
            TextView fatValue = dialog.findViewById(R.id.fat_value);
            TextView natriumValue = dialog.findViewById(R.id.natrium_value);

            infoProductName.setText(productName);
            capacityValue.setText(productVolume);
            calorieValue.setText(productCalory);
            carbohydrateValue.setText(productCarbohydrate);
            proteinValue.setText(productProtein);
            fatValue.setText(productFat);
            natriumValue.setText(productSodium);
        }
    }
}

