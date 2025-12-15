import axios from "axios";

const api = axios.create({
    baseURL:"https://sdd-hackathon-production.up.railway.app/"
})

export default  api